"""Closed loop position tracking controller.

    /odom          nav_msgs/Odometry           where the robot is
    /new_position  geometry_msgs/Pose          where it should be
    /cmd_vel       geometry_msgs/TwistStamped  what it is told to do

The node owns the ROS side only: subscriptions, the control timer, message types,
the header. The control law lives in nid_controller.py behind the Controller
interface in controller_base.py.
"""

from geometry_msgs.msg import Pose, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.exceptions import InvalidHandle
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .controller_base import Pose2D
from .geometry import yaw_from_quaternion
from .nid_controller import GOAL_DEFINES_OFFSET_POINT, NIDController


class ControllerNode(Node):
    """Drives the robot to the position most recently published on /new_position."""

    def __init__(self) -> None:
        super().__init__('controller_node')

        # Defaults match config/controller.yaml, so the node still runs sensibly if
        # a parameter file is missing rather than crashing.
        self.declare_parameter('control_period', 0.05)
        self.declare_parameter('nid_offset_L', 0.10)
        self.declare_parameter('k_p_position', 0.8)
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('goal_defines', GOAL_DEFINES_OFFSET_POINT)
        self.declare_parameter('cmd_frame_id', 'base_link')
        self.declare_parameter('odom_timeout', 0.5)
        self.declare_parameter('robot.max_linear_velocity', 0.22)
        self.declare_parameter('robot.max_angular_velocity', 2.84)

        control_period = self.get_parameter('control_period').value
        self.cmd_frame_id = self.get_parameter('cmd_frame_id').value
        self.odom_timeout = self.get_parameter('odom_timeout').value
        max_v = self.get_parameter('robot.max_linear_velocity').value
        max_w = self.get_parameter('robot.max_angular_velocity').value

        self.controller = NIDController(
            offset_L=self.get_parameter('nid_offset_L').value,
            k_p=self.get_parameter('k_p_position').value,
            max_linear_velocity=max_v,
            max_angular_velocity=max_w,
            goal_tolerance=self.get_parameter('goal_tolerance').value,
            goal_defines=self.get_parameter('goal_defines').value,
        )

        # None until the first message of each kind arrives: no guessing where the
        # robot is or where it should go.
        self.pose = None
        self.last_odom_time = None
        self.goal = None
        self.goal_reached_announced = False
        self.stopped = True

        # Odometry is sensor data: best effort, small queue, latest wins.
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        # The setpoint is published rarely and every one matters, so it gets the
        # reliable default queue.
        self.create_subscription(Pose, '/new_position', self.on_goal, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # The control law runs on its own timer, not inside a sensor callback, so
        # its rate is decoupled from odometry rate.
        self.timer = self.create_timer(control_period, self.on_control_tick)

        self.get_logger().info(
            'controller_node up. NID L=%.3f m, k_p=%.2f 1/s, tolerance=%.3f m, '
            'limits %.2f m/s and %.2f rad/s, goal_defines=%s, %.0f Hz'
            % (
                self.controller.offset_L,
                self.controller.k_p,
                self.controller.goal_tolerance,
                max_v,
                max_w,
                self.controller.goal_defines,
                1.0 / control_period,
            )
        )

    def on_odom(self, msg: Odometry) -> None:
        """Store the pose: x, y, and yaw recovered from the quaternion."""
        q = msg.pose.pose.orientation
        self.pose = Pose2D(
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            theta=yaw_from_quaternion(q.x, q.y, q.z, q.w),
        )
        self.last_odom_time = self.get_clock().now()

    def on_goal(self, msg: Pose) -> None:
        """Read the setpoint. Position only; orientation is ignored.

        NID controls the position of a point, not a heading, so the robot arrives
        pointing whichever way it had to point to get there.
        """
        new_goal = (msg.position.x, msg.position.y)
        if self.goal is None or new_goal != self.goal:
            self.goal_reached_announced = False
        self.goal = new_goal
        self.controller.reset()

    def odom_is_stale(self) -> bool:
        """Report whether odometry has stopped arriving.

        Steering on a pose that is half a second old is worse than not steering.
        """
        if self.last_odom_time is None:
            return True
        age = (self.get_clock().now() - self.last_odom_time).nanoseconds * 1e-9
        return age > self.odom_timeout

    def publish(self, v: float, omega: float) -> None:
        """Publish one velocity command, header stamped from the node clock."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cmd_frame_id
        msg.twist.linear.x = v
        msg.twist.angular.z = omega
        self.cmd_pub.publish(msg)

    def stop(self, reason: str) -> None:
        """Command zero velocity once, and say why the first time."""
        if not self.stopped:
            self.get_logger().warn('stopping: %s' % reason)
        self.stopped = True
        self.publish(0.0, 0.0)

    def on_control_tick(self) -> None:
        """One pass of the control loop, at control_period."""
        if self.pose is None:
            # Nothing published yet, not even zero, so a legitimate teleop command
            # is not fought.
            return
        if self.odom_is_stale():
            self.stop('odometry older than %.2f s' % self.odom_timeout)
            return
        if self.goal is None:
            return

        goal_x, goal_y = self.goal
        command = self.controller.compute(self.pose, goal_x, goal_y)
        _, _, distance = self.controller.tracking_error(self.pose, goal_x, goal_y)

        if command.v == 0.0 and command.omega == 0.0:
            if not self.goal_reached_announced:
                self.get_logger().info(
                    'goal reached: error %.3f m, inside tolerance %.3f m'
                    % (distance, self.controller.goal_tolerance)
                )
                self.goal_reached_announced = True
            self.stopped = True
            self.publish(0.0, 0.0)
            return

        self.stopped = False
        self.publish(command.v, command.omega)

        # One line per second while moving, not one per tick.
        self.get_logger().info(
            'error %.3f m, v %.3f m/s, omega %.3f rad/s' % (distance, command.v, command.omega),
            throttle_duration_sec=1.0,
        )

    def stop_if_possible(self) -> None:
        """Best-effort zero command on the way out.

        Not a safety guarantee: on Ctrl-C the context is often already invalid by
        the time this runs, and even when it isn't, this assumes turtlebot3_node
        stops the wheels once /cmd_vel goes quiet.
        """
        try:
            if rclpy.ok():
                self.publish(0.0, 0.0)
        except (RCLError, InvalidHandle):
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # rclpy raises ExternalShutdownException on Ctrl-C, not KeyboardInterrupt.
        pass
    finally:
        node.stop_if_possible()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
