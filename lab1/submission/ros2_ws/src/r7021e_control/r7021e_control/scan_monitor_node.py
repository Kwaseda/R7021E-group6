"""Closest point on the wall, from the laser scan. Task 3 of Lab 1.

    /scan                sensor_msgs/LaserScan       what the lidar sees
    /closest_wall_point  geometry_msgs/PointStamped  where the nearest wall is

Its own node rather than a branch inside the controller, since the controller has no
use for this number and the plot needs it at scan rate for a whole run.

Published in the scan's own frame, untransformed -- the honest frame for a raw
measurement. Anything that wants it in odom can use tf, recorded in the bag
alongside it.

Selection maths is in scan_utils.py, shared with the wall follower so both nodes
agree on what counts as a valid beam.
"""

import math

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .scan_utils import closest_point


class ScanMonitorNode(Node):
    """Publishes the nearest valid lidar return, once per scan."""

    def __init__(self) -> None:
        super().__init__('scan_monitor_node')

        # Range gating comes from config/robot.yaml, since it is a fact about the
        # LDS-01 rather than a choice this node makes.
        self.declare_parameter('robot.scan_range_min', 0.12)
        self.declare_parameter('robot.scan_range_max', 3.5)
        self.declare_parameter('output_frame_id', '')
        self.declare_parameter('invalid_scan_warn_period', 5.0)

        self.range_min = self.get_parameter('robot.scan_range_min').value
        self.range_max = self.get_parameter('robot.scan_range_max').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        self.warn_period = self.get_parameter('invalid_scan_warn_period').value

        self.point_pub = self.create_publisher(PointStamped, '/closest_wall_point', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        self.get_logger().info(
            'scan_monitor_node up. valid range gate %.2f to %.2f m'
            % (self.range_min, self.range_max)
        )

    def on_scan(self, msg: LaserScan) -> None:
        """Find the closest valid beam and publish it as a point."""
        found = closest_point(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            self.range_min,
            self.range_max,
        )

        if found is None:
            # Not an error: a robot in open ground with every wall beyond range has
            # no closest wall. Publishing a zero here would put a phantom obstacle
            # at the robot's own origin.
            self.get_logger().warn(
                'no valid beam in this scan: every range was outside %.2f to %.2f m, '
                'or was inf or nan'
                % (self.range_min, self.range_max),
                throttle_duration_sec=self.warn_period,
            )
            return

        distance, bearing, _ = found

        out = PointStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.output_frame_id or msg.header.frame_id
        # Polar to Cartesian in the sensor frame.
        out.point.x = distance * math.cos(bearing)
        out.point.y = distance * math.sin(bearing)
        out.point.z = 0.0
        self.point_pub.publish(out)

        self.get_logger().info(
            'closest wall %.3f m at %.1f degrees' % (distance, math.degrees(bearing)),
            throttle_duration_sec=1.0,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanMonitorNode()
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
