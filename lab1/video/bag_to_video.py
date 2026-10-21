#!/usr/bin/env python3
"""Render an offline Lab 1 visualization from a rosbag; this is not RViz capture.

    source /opt/ros/jazzy/setup.bash
    python3 lab1/video/bag_to_video.py lab1/submission/bags/task1-position-1m-forward \
        --out lab1/submission/videos/bag-replay-task1.mp4

Each frame shows the odometry trail, current robot position, the latest recorded
goal (when present), and the latest LaserScan in its sensor frame.
"""

import argparse
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
import numpy as np
import rosbag2_py
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan


TYPES = {
    '/odom': Odometry,
    '/new_position': Pose,
    '/scan': LaserScan,
}


def read_bag(uri):
    """Read stored odometry, goals, and scans with the bag storage timestamps."""
    storage = 'mcap' if any(name.endswith('.mcap') for name in os.listdir(uri)) else 'sqlite3'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=storage),
                rosbag2_py.ConverterOptions('', ''))
    events = []
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic in TYPES:
            events.append((stamp * 1e-9, topic,
                           deserialize_message(raw, TYPES[topic])))
    if not events:
        raise ValueError(f'No supported messages in bag: {uri}')
    return events


def pose_xyyaw(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return p.x, p.y, yaw


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='bag directory containing metadata.yaml')
    parser.add_argument('--out', required=True, help='output .mp4 path')
    parser.add_argument('--speed', type=float, default=8.0,
                        help='bag seconds per video second')
    parser.add_argument('--fps', type=int, default=10)
    args = parser.parse_args()
    bag = os.path.expanduser(args.bag).rstrip('/')
    out = os.path.expanduser(args.out)
    if args.speed <= 0 or args.fps <= 0:
        parser.error('--speed and --fps must be positive')
    if not out.lower().endswith('.mp4'):
        parser.error('--out must end in .mp4')

    events = read_bag(bag)
    t0, t1 = events[0][0], events[-1][0]
    odom = [(t, msg) for t, topic, msg in events if topic == '/odom']
    goals = [(t, msg.position.x, msg.position.y)
             for t, topic, msg in events if topic == '/new_position']
    scans = [(t, msg) for t, topic, msg in events if topic == '/scan']
    if len(odom) < 2:
        raise ValueError('At least two /odom messages are required for a path.')
    if not scans:
        raise ValueError('No /scan messages in bag; cannot render laser evidence.')

    path_xy = np.array([[msg.pose.pose.position.x, msg.pose.pose.position.y]
                        for _, msg in odom])
    goal_xy = np.array([[x, y] for _, x, y in goals]) if goals else np.empty((0, 2))
    goal_times = np.array([t for t, _, _ in goals])
    all_xy = np.vstack((path_xy, goal_xy)) if len(goal_xy) else path_xy
    margin = 0.5
    xlim = (float(all_xy[:, 0].min() - margin), float(all_xy[:, 0].max() + margin))
    ylim = (float(all_xy[:, 1].min() - margin), float(all_xy[:, 1].max() + margin))
    step = args.speed / args.fps
    frames = np.arange(t0, t1 + step * 0.5, step)
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    print(f'{len(odom)} odometry, {len(scans)} scans, {len(goals)} goals; '
          f'{t1 - t0:.1f} bag seconds -> {len(frames)} frames at {out}')

    fig, (world, laser) = plt.subplots(1, 2, figsize=(12, 6))
    fig.subplots_adjust(top=0.88, bottom=0.12, wspace=0.28)
    writer = FFMpegWriter(fps=args.fps, metadata={'title': 'Lab 1 bag visualization'})
    odom_times = np.array([t for t, _ in odom])
    scan_times = np.array([t for t, _ in scans])
    with writer.saving(fig, out, dpi=110):
        oi = gi = si = 0
        latest_goal = None
        latest_scan = None
        for now in frames:
            while oi + 1 < len(odom) and odom_times[oi + 1] <= now:
                oi += 1
            while gi < len(goals) and goal_times[gi] <= now:
                latest_goal = goals[gi][1:]
                gi += 1
            while si < len(scans) and scan_times[si] <= now:
                latest_scan = scans[si][1]
                si += 1
            if latest_scan is None:
                continue

            world.clear()
            trail = path_xy[:oi + 1]
            world.plot(trail[:, 0], trail[:, 1], color='#1565c0', lw=1.8,
                       label='odometry path')
            world.plot(trail[-1, 0], trail[-1, 1], 'o', color='#1565c0',
                       ms=9, label='current robot position')
            world.plot(path_xy[0, 0], path_xy[0, 1], 'o', color='#17a2a4',
                       ms=7, label='start')
            visible_goals = goal_xy[goal_times <= now]
            if len(visible_goals):
                world.scatter(visible_goals[:, 0], visible_goals[:, 1], s=24,
                              facecolors='none', edgecolors='#88a96b',
                              label='published goal history')
            if latest_goal is not None:
                world.plot(*latest_goal, marker='*', color='#2c9b45', ms=15,
                           label='current goal')
            elif goals:
                world.text(0.02, 0.98, 'No /new_position received yet',
                           transform=world.transAxes, va='top', fontsize=9,
                           bbox={'facecolor': 'white', 'alpha': 0.85,
                                 'edgecolor': '0.7'})
            world.set_xlim(*xlim)
            world.set_ylim(*ylim)
            world.set_aspect('equal', adjustable='box')
            world.set_xlabel('odom x [m]')
            world.set_ylabel('odom y [m]')
            world.grid(alpha=0.25)
            world.legend(loc='best', fontsize=8)
            if not goals:
                world.text(0.02, 0.98, 'No goal recorded (reactive run)',
                           transform=world.transAxes, va='top', fontsize=9,
                           bbox={'facecolor': 'white', 'alpha': 0.85,
                                 'edgecolor': '0.7'})

            laser.clear()
            scan = latest_scan
            ranges = np.asarray(scan.ranges, dtype=float)
            angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
            valid = (np.isfinite(ranges) & (ranges >= scan.range_min)
                     & (ranges <= scan.range_max))
            indices = np.flatnonzero(valid)
            if len(indices) > 600:
                indices = indices[::int(np.ceil(len(indices) / 600))]
            laser.scatter(ranges[indices] * np.cos(angles[indices]),
                          ranges[indices] * np.sin(angles[indices]),
                          s=5, color='#d33b32', alpha=0.8, label='current scan')
            laser.plot(0, 0, marker='o', color='#1565c0', ms=7,
                       label='laser-frame origin')
            max_range = max(1.0, float(scan.range_max))
            laser.set_xlim(-max_range, max_range)
            laser.set_ylim(-max_range, max_range)
            laser.set_aspect('equal', adjustable='box')
            laser.set_xlabel('laser-frame x [m]')
            laser.set_ylabel('laser-frame y [m]')
            laser.grid(alpha=0.25)
            laser.legend(loc='best', fontsize=8)
            laser.set_title('Latest /scan in sensor frame')
            x, y, _ = pose_xyyaw(odom[oi][1])
            fig.suptitle(f'Lab 1 offline bag visualization — {os.path.basename(bag)}'
                         f' — t = {now - t0:.1f} s — position ({x:.2f}, {y:.2f}) m')
            fig.text(0.5, 0.025,
                     'Generated from bag data; not an RViz screen recording.',
                     ha='center', fontsize=9, color='0.35')
            writer.grab_frame()
    plt.close(fig)
    print(f'Wrote {out}')


if __name__ == '__main__':
    main()
