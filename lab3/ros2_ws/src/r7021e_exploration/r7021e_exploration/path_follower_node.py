#!/usr/bin/env python3
# frontier_detector_node.py

import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Path
from geometry_msgs.msg import TwistStamped, TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.path = []
        self.path_header=""

        # Parameters
        self.declare_parameter('max_v', 0.15)  # m/s
        self.declare_parameter('kp_vel', 1.0)
        self.declare_parameter('max_w', 0.6)  # rad/s, faster turns slip in Gazebo
        self.declare_parameter('kp_yaw', 1.0)  # keeps turns while driving under 0.3 rad/s
        self.declare_parameter('look_ahead', 0.12)  # m, about one waypoint; 0.2 cut corners into walls
        # scan check: forward speed falls to zero as something enters the strip
        self.declare_parameter('stop_distance', 0.18)  # m, forward speed is zero here
        self.declare_parameter('slow_distance', 0.30)  # m, full speed from here
        self.declare_parameter('half_width', 0.10)  # m, half the strip the body sweeps
        self.declare_parameter('scan_timeout', 0.5)  # s, older scans count as missing
        self.declare_parameter('goal_tolerance', 0.05)  # m, stop at the last waypoint
        # blocked ahead with the right heading for this long: back off, if the rear is clear
        self.declare_parameter('blocked_time', 2.0)  # s
        self.declare_parameter('backoff_distance', 0.10)  # m
        self.declare_parameter('backoff_speed', 0.05)  # m/s

        self.max_v = float(self.get_parameter('max_v').value)
        self.kp_vel = float(self.get_parameter('kp_vel').value)
        self.max_w = float(self.get_parameter('max_w').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.look_ahead = float(self.get_parameter('look_ahead').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.slow_distance = float(self.get_parameter('slow_distance').value)
        self.half_width = float(self.get_parameter('half_width').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.blocked_time = float(self.get_parameter('blocked_time').value)
        self.backoff_distance = float(self.get_parameter('backoff_distance').value)
        self.backoff_speed = float(self.get_parameter('backoff_speed').value)
        self.scan = None
        self.scan_rx = None
        self.blocked_since = None   # when the strip ahead first blocked a wanted move
        self.backoff_end = None     # when the current back-off stops

        # subscriptions and publishers
        self.map_sub = self.create_subscription(
            Path, 'path', self.path_callback, 1)

        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)

        self.vel_pub = self.create_publisher(
            TwistStamped, 'cmd_vel', 1)

        # TransformListener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # Timers
        self.follow_path_time = self.create_timer(0.1, self.follow_path)

    def path_callback(self, msg: Path):
        self.path_header=msg.header.frame_id
        self.path=[]
        for point in msg.poses:
            self.path.append((point.pose.position.x, point.pose.position.y))

    def scan_callback(self, msg: LaserScan):
        self.scan = msg
        self.scan_rx = self.get_clock().now()

    def forward_clearance(self, sign=1.0):
        '''
        Distance to the nearest return in the strip ahead of the robot
        (sign=-1.0: the strip behind it).
        Returns None when there is no fresh scan. The scan frame is treated
        as base_link (the laser sits 3 cm behind the centre).
        '''
        if self.scan is None:
            return None
        age = (self.get_clock().now() - self.scan_rx).nanoseconds * 1e-9
        if age > self.scan_timeout:
            return None
        r = np.asarray(self.scan.ranges, dtype=float)
        a = self.scan.angle_min + self.scan.angle_increment * np.arange(len(r))
        valid = np.isfinite(r) & (r > self.scan.range_min) & (r < self.scan.range_max)
        r, a = r[valid], a[valid]
        x = sign * r * np.cos(a)
        y = r * np.sin(a)
        ahead = (x > 0.0) & (np.abs(y) <= self.half_width)
        return float(x[ahead].min()) if ahead.any() else math.inf

    def update_robot_pos(self):
        try:
            ts: TransformStamped = self.buffer.lookup_transform(
                'map',         # target frame
                'base_link',   # source frame
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return False

        t = ts.transform.translation
        self.robot_pos = (t.x, t.y)

        q = ts.transform.rotation
        self.robot_yaw = self.yaw_from_quaternion(q.x, q.y, q.z, q.w)
        return True

    def yaw_from_quaternion(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def follow_path(self):
        if not self.path:
            return

        vel_msg = TwistStamped()
        vel_msg.header.stamp= self.get_clock().now().to_msg()
        vel_msg.header.frame_id= self.path_header

        if not self.update_robot_pos():
            return

        while len(self.path) > 1:
            if self.dist(self.path[0], self.robot_pos) > self.look_ahead:
                break
            self.path.pop(0)

        if len(self.path) == 1 and self.dist(self.path[0], self.robot_pos) < self.goal_tolerance:
            self.vel_pub.publish(vel_msg)  # at the end of the path: publish zero
            return

        dist_to_target = self.dist(self.path[0], self.robot_pos)
        if dist_to_target < self.look_ahead:
            vel_msg.twist.linear.x = self.kp_vel*dist_to_target
            vel_msg.twist.linear.x = min(self.max_v, vel_msg.twist.linear.x)
        else:
            vel_msg.twist.linear.x = self.max_v

        dx = self.path[0][0]-self.robot_pos[0]
        dy = self.path[0][1]-self.robot_pos[1]
        ang_to_target = math.atan2(dy, dx)
        dif_ang = self.ang_dist(self.robot_yaw, ang_to_target)

        if abs(dif_ang)>0.3:
            vel_msg.twist.linear.x = 0.0

        now = self.get_clock().now()
        if self.backoff_end is not None:
            rear = self.forward_clearance(-1.0)
            if now < self.backoff_end and rear is not None and rear > self.stop_distance:
                vel_msg.twist.linear.x = -self.backoff_speed   # straight back, no turn
                self.vel_pub.publish(vel_msg)
                return
            self.backoff_end = None

        clearance = self.forward_clearance()
        if clearance is None:
            vel_msg.twist.linear.x = 0.0
            self.get_logger().warn('no fresh scan, forward speed held at zero',
                                   throttle_duration_sec=5.0)
        else:
            gap = (clearance - self.stop_distance) / (self.slow_distance - self.stop_distance)
            vel_msg.twist.linear.x *= min(1.0, max(0.0, gap))

        # Blocked: we want to go forward, the heading is right, and the strip is full.
        if clearance is not None and clearance <= self.stop_distance and abs(dif_ang) <= 0.3:
            if self.blocked_since is None:
                self.blocked_since = now
            elif (now - self.blocked_since).nanoseconds * 1e-9 > self.blocked_time:
                self.get_logger().warn(f'blocked ahead at {clearance:.2f} m, backing off')
                self.backoff_end = now + Duration(
                    seconds=self.backoff_distance / self.backoff_speed)
                self.blocked_since = None
        else:
            self.blocked_since = None
        vel_msg.twist.angular.z = dif_ang * self.kp_yaw
        if abs(vel_msg.twist.angular.z) > self.max_w:
            vel_msg.twist.angular.z = math.copysign(self.max_w, vel_msg.twist.angular.z)

        self.vel_pub.publish(vel_msg)

    def dist(self, pos1, pos2):
        dx = pos1[0] - pos2[0]
        dy = pos1[1] - pos2[1]

        return math.sqrt(dx*dx+dy*dy)

    def ang_dist(self, v1: float, v2: float):
        '''
        Get the  shortest distance between two angles,
        handles extreme values of angles.
        '''
        v1 = math.copysign(math.fmod(v1, (2*math.pi)), v1)
        v2 = math.copysign(math.fmod(v2, (2*math.pi)), v2)

        dif_ang = v2-v1

        if abs(dif_ang) < math.pi:
            return dif_ang
        else:
            dif_ang2 = 2*math.pi-abs(dif_ang)
            dif_ang2 = math.copysign(dif_ang2, -dif_ang)
            return dif_ang2


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
