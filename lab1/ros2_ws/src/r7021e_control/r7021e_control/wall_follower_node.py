"""Wall following until the loop closes. Task 4 of Lab 1.

    /scan     sensor_msgs/LaserScan       what the lidar sees
    /odom     nav_msgs/Odometry           used only to notice the loop has closed
    /cmd_vel  geometry_msgs/TwistStamped  what the robot is told to do

"Based on the LIDAR values, make the robot follow the walls until it completes a
loop."

This node and controller_node both publish to /cmd_vel, so only one runs at a time --
enforced by the launch file's `mode` argument, since two publishers on one topic
interleave rather than error.

Odometry steers nothing here; it is read only to detect that the odometry estimate
has returned to the start pose. The law itself is in wall_following.py.
"""

import math

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.exceptions import InvalidHandle
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .geometry import yaw_from_quaternion
from .scan_utils import beam_at, closest_point, sector_min
from .wall_following import (
    LapStart,
    LoopCloseDetector,
    max_fittable_wall_angle,
    wall_geometry,
    WallFollowLaw,
)


class WallFollowerNode(Node):
    """Follows the wall on one side at a fixed distance until a lap closes."""

    def __init__(self) -> None:
        super().__init__('wall_follower_node')

        # Defaults match config/wall_follower.yaml, with physical limits defaulting
        # to the Burger numbers in config/robot.yaml.
        self.declare_parameter('follow_side', 'right')
        self.declare_parameter('wall_follow_setpoint', 0.5)
        self.declare_parameter('k_p_wall', 1.5)
        self.declare_parameter('k_heading_wall', 1.0)
        self.declare_parameter('wall_beam_separation_deg', 40.0)
        self.declare_parameter('wall_beam_window_deg', 5.0)
        self.declare_parameter('max_wall_angle_deg', 60.0)
        self.declare_parameter('wall_follow_speed', 0.15)
        self.declare_parameter('front_stop_distance', 0.35)
        self.declare_parameter('corner_turn_rate', 1.0)
        self.declare_parameter('max_distance_error', 0.5)
        self.declare_parameter('acquire_turn_gain', 1.0)
        self.declare_parameter('side_sector_width_deg', 60.0)
        self.declare_parameter('front_sector_width_deg', 60.0)
        self.declare_parameter('control_period', 0.05)
        self.declare_parameter('loop_close_tolerance', 0.2)
        self.declare_parameter('loop_min_distance', 2.0)
        self.declare_parameter('loop_close_heading_deg', 45.0)
        self.declare_parameter('lap_settle_seconds', 0.6)
        self.declare_parameter('lap_settle_angle_deg', 15.0)
        self.declare_parameter('lap_settle_distance', 0.15)
        self.declare_parameter('stop_on_loop_close', True)
        self.declare_parameter('odom_timeout', 0.5)
        self.declare_parameter('cmd_frame_id', 'base_link')
        self.declare_parameter('robot.max_linear_velocity', 0.22)
        self.declare_parameter('robot.max_angular_velocity', 2.84)
        self.declare_parameter('robot.scan_range_min', 0.12)
        self.declare_parameter('robot.scan_range_max', 3.5)

        self.law = WallFollowLaw(
            follow_side=self.get_parameter('follow_side').value,
            setpoint=self.get_parameter('wall_follow_setpoint').value,
            k_p=self.get_parameter('k_p_wall').value,
            k_heading=self.get_parameter('k_heading_wall').value,
            speed=self.get_parameter('wall_follow_speed').value,
            front_stop_distance=self.get_parameter('front_stop_distance').value,
            corner_turn_rate=self.get_parameter('corner_turn_rate').value,
            max_distance_error=self.get_parameter('max_distance_error').value,
            acquire_turn_gain=self.get_parameter('acquire_turn_gain').value,
            max_linear_velocity=self.get_parameter('robot.max_linear_velocity').value,
            max_angular_velocity=self.get_parameter('robot.max_angular_velocity').value,
        )
        self.loop_detector = LoopCloseDetector(
            tolerance=self.get_parameter('loop_close_tolerance').value,
            min_distance=self.get_parameter('loop_min_distance').value,
            heading_tolerance=math.radians(
                self.get_parameter('loop_close_heading_deg').value),
        )
        control_period = self.get_parameter('control_period').value
        self.lap_start = LapStart(
            setpoint=self.get_parameter('wall_follow_setpoint').value,
            distance_tolerance=self.get_parameter('lap_settle_distance').value,
            angle_tolerance=math.radians(
                self.get_parameter('lap_settle_angle_deg').value),
            settle_ticks=round(
                self.get_parameter('lap_settle_seconds').value / control_period),
        )

        # Sector widths are written in degrees in YAML, since that is how anyone
        # reasons about a lidar sector, and converted once, here.
        self.side_width = math.radians(self.get_parameter('side_sector_width_deg').value)
        self.beam_separation = math.radians(
            self.get_parameter('wall_beam_separation_deg').value)
        self.beam_window = math.radians(
            self.get_parameter('wall_beam_window_deg').value)

        # The fit is singular at 90 - separation degrees, where the forward beam runs
        # parallel to the wall. Admitting angles near it is what lets a forward beam
        # that missed a convex obstacle and flew on to something metres behind be read
        # as a near wall raking away, so the accepted range is capped strictly inside
        # the singularity whatever the YAML asks for.
        requested = math.radians(self.get_parameter('max_wall_angle_deg').value)
        usable = 0.8 * max_fittable_wall_angle(self.beam_separation)
        self.max_wall_angle = min(requested, usable)
        if requested > usable:
            self.get_logger().warn(
                'max_wall_angle_deg %.1f is too close to the %.1f degree singularity '
                'for a %.1f degree beam separation; capping the fit at %.1f degrees'
                % (math.degrees(requested),
                   math.degrees(max_fittable_wall_angle(self.beam_separation)),
                   math.degrees(self.beam_separation),
                   math.degrees(self.max_wall_angle))
            )
        self.front_width = math.radians(self.get_parameter('front_sector_width_deg').value)
        self.range_min = self.get_parameter('robot.scan_range_min').value
        self.range_max = self.get_parameter('robot.scan_range_max').value
        self.stop_on_loop_close = self.get_parameter('stop_on_loop_close').value
        self.odom_timeout = self.get_parameter('odom_timeout').value
        self.cmd_frame_id = self.get_parameter('cmd_frame_id').value

        self.scan = None
        self.position = None
        self.last_odom_time = None
        self.finished = False
        # Lap starts when the robot reaches a wall, not when this node starts.
        self.following_started = False

        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Runs on its own timer, not the scan callback, so control rate is decoupled
        # from lidar rate (which differs between sim and the real robot).
        self.timer = self.create_timer(control_period, self.on_control_tick)

        self.get_logger().info(
            'wall_follower_node up. following the %s wall at %.2f m, k_p %.2f, '
            'speed %.2f m/s, front stop %.2f m, loop closes within %.2f m after %.1f m'
            % (
                self.law.follow_side,
                self.law.setpoint,
                self.law.k_p,
                self.law.speed,
                self.law.front_stop_distance,
                self.loop_detector.tolerance,
                self.loop_detector.min_distance,
            )
        )

    def on_scan(self, msg: LaserScan) -> None:
        """Keep the latest scan. The control tick reads it, not this callback."""
        self.scan = msg

    def on_odom(self, msg: Odometry) -> None:
        """Track the pose for the loop closure test only. Nothing here steers."""
        q = msg.pose.pose.orientation
        self.position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quaternion(q.x, q.y, q.z, q.w),
        )
        self.last_odom_time = self.get_clock().now()

    def publish(self, v: float, omega: float) -> None:
        """Publish one velocity command, header stamped from the node clock."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cmd_frame_id
        msg.twist.linear.x = v
        msg.twist.angular.z = omega
        self.cmd_pub.publish(msg)

    def odom_is_stale(self) -> bool:
        """Report whether odometry has stopped arriving."""
        if self.last_odom_time is None:
            return True
        age = (self.get_clock().now() - self.last_odom_time).nanoseconds * 1e-9
        return age > self.odom_timeout

    def wall_geometry_from(self, scan):
        """Perpendicular distance and wall angle from two beams, or None.

        Two specific rays, not the minimum over a sector, since the geometry needs
        to know which ray produced which range.
        """
        # Forward beam is toward the front, opposite sign to side_sign.
        forward = self.law.side_bearing - self.law.side_sign * self.beam_separation
        a = beam_at(scan.ranges, scan.angle_min, scan.angle_increment,
                    self.range_min, self.range_max, self.law.side_bearing,
                    self.beam_window)
        b = beam_at(scan.ranges, scan.angle_min, scan.angle_increment,
                    self.range_min, self.range_max, forward, self.beam_window)
        if a is None or b is None:
            return None

        distance, psi = wall_geometry(a[0], b[0], self.beam_separation)
        # Reject a fit that says the wall runs almost straight at the robot -- two
        # beams hitting different walls at a corner produce exactly that.
        if abs(psi) > self.max_wall_angle or distance <= 0.0:
            return None
        return distance, psi

    def on_control_tick(self) -> None:
        """One pass of the wall following loop, at control_period."""
        if self.finished:
            # Keep publishing zero rather than falling silent, so nothing
            # downstream holds the last non-zero command.
            self.publish(0.0, 0.0)
            return

        if self.scan is None:
            return

        side = sector_min(
            self.scan.ranges,
            self.scan.angle_min,
            self.scan.angle_increment,
            self.range_min,
            self.range_max,
            self.law.side_bearing,
            self.side_width,
        )
        front = sector_min(
            self.scan.ranges,
            self.scan.angle_min,
            self.scan.angle_increment,
            self.range_min,
            self.range_max,
            0.0,
            self.front_width,
        )

        # Nearest beam anywhere in the scan: distinguishes "wall briefly out of the
        # side sector" from "no wall nearby." Same function scan_monitor_node uses.
        nearest = closest_point(
            self.scan.ranges,
            self.scan.angle_min,
            self.scan.angle_increment,
            self.range_min,
            self.range_max,
        )

        side_distance = side[0] if side is not None else None
        front_distance = front[0] if front is not None else None
        geometry = self.wall_geometry_from(self.scan)
        command, state = self.law.compute(
            side_distance, front_distance,
            None if nearest is None else (nearest[0], nearest[1]),
            geometry,
        )

        # The lap reference is the pose the loop test measures against, so it has to be
        # a pose on the track the follower settles into -- at the setpoint distance and
        # running parallel, held for long enough to be past the approach transient.
        # Neither the node's start pose (the robot starts mid-room and never returns
        # there) nor the first following tick (right distance, wrong heading) is one.
        if not self.following_started and self.lap_start.update(
                state, side_distance, geometry):
            self.following_started = True
            self.get_logger().info(
                'lap reference set: settled on the %s wall at %.3f m, %.1f degrees off '
                'parallel' % (self.law.follow_side, side_distance,
                              math.degrees(geometry[1]))
            )

        if self.following_started and self.position is not None and not self.odom_is_stale():
            if self.loop_detector.update(*self.position):
                error = self.loop_detector.error_to_start(*self.position[:2])
                self.get_logger().info(
                    'loop closed: back within %.3f m of where wall following began, '
                    'after %.2f m of travel (tolerance %.2f m)'
                    % (error, self.loop_detector.travelled, self.loop_detector.tolerance)
                )
                if self.stop_on_loop_close:
                    self.finished = True
                    self.publish(0.0, 0.0)
                    return

        self.publish(command.v, command.omega)

        self.get_logger().info(
            '%s: side %s m, front %s m, v %.3f m/s, omega %.3f rad/s'
            % (
                state,
                'none' if side_distance is None else '%.3f' % side_distance,
                'none' if front_distance is None else '%.3f' % front_distance,
                command.v,
                command.omega,
            ),
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
    node = WallFollowerNode()
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
