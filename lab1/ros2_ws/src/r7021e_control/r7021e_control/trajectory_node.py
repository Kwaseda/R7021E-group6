"""Figure of eight setpoint generator.

    /new_position  geometry_msgs/Pose  the goal the controller chases

Published here rather than generated inside the controller, so /new_position stays
the single control entry point: a terminal setpoint, this node, or a later planner
all drive the same controller through the same message.

The curve itself is in trajectories.py.
"""

import math

from geometry_msgs.msg import Pose
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .geometry import quaternion_from_yaw
from .trajectories import FigureEightTrajectory


class TrajectoryNode(Node):
    """Publishes the figure of eight as a stream of position setpoints."""

    def __init__(self) -> None:
        super().__init__('trajectory_node')

        # Defaults match config/trajectory.yaml.
        self.declare_parameter('width', 1.0)
        self.declare_parameter('height', 0.5)
        self.declare_parameter('centre_x', 0.0)
        self.declare_parameter('centre_y', 0.0)
        self.declare_parameter('lap_period', 50.0)
        self.declare_parameter('laps', 2)
        self.declare_parameter('publish_period', 0.05)
        self.declare_parameter('start_delay', 3.0)

        self.trajectory = FigureEightTrajectory(
            width=self.get_parameter('width').value,
            height=self.get_parameter('height').value,
            centre_x=self.get_parameter('centre_x').value,
            centre_y=self.get_parameter('centre_y').value,
            lap_period=self.get_parameter('lap_period').value,
            laps=self.get_parameter('laps').value,
            start_delay=self.get_parameter('start_delay').value,
        )

        self.goal_pub = self.create_publisher(Pose, '/new_position', 10)
        self.finished_announced = False

        # Clock starts on the first tick, not here: under use_sim_time the clock
        # reads zero until the first /clock message arrives, so a start time
        # captured here would put lap one in the past.
        self.start_time = None
        publish_period = self.get_parameter('publish_period').value
        self.timer = self.create_timer(publish_period, self.on_publish_tick)

        self.get_logger().info(
            'trajectory_node up. figure of eight %.2f by %.2f m about (%.2f, %.2f), '
            '%.1f s per lap, %d laps, %.1f s start delay, %.0f Hz'
            % (
                2.0 * self.trajectory.width,
                2.0 * self.trajectory.height,
                self.trajectory.centre_x,
                self.trajectory.centre_y,
                self.trajectory.lap_period,
                self.trajectory.laps,
                self.trajectory.start_delay,
                1.0 / publish_period,
            )
        )

    def on_publish_tick(self) -> None:
        """Publish the setpoint for the current time."""
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        x, y, finished = self.trajectory.point_at(elapsed)

        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        # Orientation set to the path's tangent rather than left as an invalid
        # all-zero quaternion. The controller ignores it; this is so a bag of this
        # topic can still be drawn by other tools.
        tangent = self.heading_at(elapsed)
        qx, qy, qz, qw = quaternion_from_yaw(tangent)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        self.goal_pub.publish(msg)

        if finished and not self.finished_announced:
            self.get_logger().info(
                '%d laps done after %.1f s, holding the final setpoint at (%.3f, %.3f)'
                % (self.trajectory.laps, elapsed, x, y)
            )
            self.finished_announced = True

    def heading_at(self, elapsed: float) -> float:
        """Direction the setpoint is travelling, radians.

        Finite difference over one publish period, since the value only fills a
        quaternion nothing steers on.
        """
        step = 0.05
        x0, y0, _ = self.trajectory.point_at(elapsed)
        x1, y1, _ = self.trajectory.point_at(elapsed + step)
        if x1 == x0 and y1 == y0:
            return 0.0
        return math.atan2(y1 - y0, x1 - x0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # rclpy raises ExternalShutdownException on Ctrl-C, not KeyboardInterrupt.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
