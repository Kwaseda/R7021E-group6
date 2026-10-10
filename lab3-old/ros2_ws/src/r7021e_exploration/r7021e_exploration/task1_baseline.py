#!/usr/bin/env python3
"""Task 1 baseline: send a Path to the follower, then send a new one at its end."""

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros


class Task1Baseline(Node):
    def __init__(self) -> None:
        super().__init__('task1_baseline')
        self.length = self.declare_parameter('length', 0.5).value        # m
        self.tolerance = self.declare_parameter('tolerance', 0.10).value  # m, end reached
        self.spacing = 0.10                                              # m between points

        self.path_pub = self.create_publisher(Path, 'path', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.start = None       # (x, y), where the robot was at the first pose
        self.end = None         # (x, y), length metres ahead of start
        self.target = None      # the end of the path we sent last
        self.legs = 0
        self.timer = self.create_timer(0.5, self._tick)

    def _pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time(),
                                                timeout=Duration(seconds=0.5))
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def _send(self, a, b) -> None:
        n = max(1, int(math.ceil(math.dist(a, b) / self.spacing)))
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        for i in range(n + 1):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = a[0] + (b[0] - a[0]) * i / n
            pose.pose.position.y = a[1] + (b[1] - a[1]) * i / n
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            msg.poses.append(pose)
        self.path_pub.publish(msg)
        self.target = b
        self.legs += 1
        self.get_logger().info(
            f'leg {self.legs}: ({a[0]:.2f}, {a[1]:.2f}) -> ({b[0]:.2f}, {b[1]:.2f})')

    def _tick(self) -> None:
        pose = self._pose()
        if pose is None:
            return
        x, y, yaw = pose
        if self.start is None:
            self.start = (x, y)
            self.end = (x + self.length * math.cos(yaw), y + self.length * math.sin(yaw))
            self._send(self.start, self.end)
        elif math.dist((x, y), self.target) < self.tolerance:
            self._send(self.target, self.start if self.target == self.end else self.end)


def main() -> None:
    rclpy.init()
    node = Task1Baseline()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
