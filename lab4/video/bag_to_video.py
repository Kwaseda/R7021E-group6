#!/usr/bin/env python3
"""Make a video of a Lab 4 run from the bag that replay_record.sh wrote. No RViz needed.

    source /opt/ros/jazzy/setup.bash
    python3 ~/R7021E-group6/lab4/video/bag_to_video.py <recorded bag> --out run.mp4 [--speed 4]

Each frame shows what RViz shows: the filter's map (/map), the particle cloud (/particles),
the estimated path (/slam_path) and the current laser scan drawn at the estimated pose.
Time comes from the bag.
"""

import argparse
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FFMpegWriter  # noqa: E402
import numpy as np  # noqa: E402
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from geometry_msgs.msg import PoseArray  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Path  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

TYPES = {'/map': OccupancyGrid, '/particles': PoseArray, '/slam_path': Path, '/scan': LaserScan,
         '/tf': TFMessage}

## base_footprint -> base_scan on the TurtleBot3 Burger (the node prints it at start).
LASER_X = -0.032


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read_bag(uri):
    """Time-sorted list of (t, topic, message)."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TYPES)))

    events = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        events.append((t * 1e-9, topic, deserialize_message(data, TYPES[topic])))

    return events


def grid_image(msg):
    g = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)

    # Same grey levels as RViz: unknown mid grey, free white, occupied black.
    img = np.full(g.shape, 0.7)
    img[(g >= 0) & (g < 50)] = 1.0
    img[g >= 50] = 0.0

    o, r = msg.info.origin.position, msg.info.resolution
    return img, [o.x, o.x + g.shape[1] * r, o.y, o.y + g.shape[0] * r]


def scan_points(scan, pose):
    """Scan endpoints in the map frame, drawn at the estimated pose."""
    r = np.array(scan.ranges)
    a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
    ok = np.isfinite(r) & (r > scan.range_min) & (r < scan.range_max)

    x, y, th = pose
    lx = x + LASER_X * math.cos(th)
    ly = y + LASER_X * math.sin(th)
    return (lx + r[ok] * np.cos(th + a[ok]),
            ly + r[ok] * np.sin(th + a[ok]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--out', required=True, help='.mp4')
    ap.add_argument('--speed', type=float, default=4.0, help='times faster than real time')
    ap.add_argument('--fps', type=int, default=10)
    ap.add_argument('--title', default=None)
    a = ap.parse_args()

    events = read_bag(a.bag)
    if not any(k == '/map' for _, k, _ in events):
        raise SystemExit('no /map in ' + a.bag)

    t0, t1 = events[0][0], events[-1][0]
    frames = np.arange(t0, t1, a.speed / a.fps)
    title = a.title or os.path.basename(os.path.normpath(a.bag))

    # Fixed view: everything the map ever covered with known cells, plus a margin.
    xs, ys = [], []
    for _, k, m in events:
        if k == '/slam_path' and m.poses:
            xs += [p.pose.position.x for p in m.poses]
            ys += [p.pose.position.y for p in m.poses]
    cx, cy = (np.mean(xs), np.mean(ys)) if xs else (0.0, 0.0)
    half = max(np.ptp(xs) if xs else 2.0, np.ptp(ys) if ys else 2.0) / 2.0 + 1.5

    fig, ax = plt.subplots(figsize=(8, 8))
    writer = FFMpegWriter(fps=a.fps)
    latest = {}
    i = 0

    # The live pose is map -> odom (the filter's correction) composed with odom -> base.
    m2o, o2b = (0.0, 0.0, 0.0), None

    with writer.saving(fig, a.out, dpi=100):
        for tf in frames:
            while i < len(events) and events[i][0] <= tf:
                t, k, m = events[i]
                if k == '/tf':
                    for tr in m.transforms:
                        p = (tr.transform.translation.x, tr.transform.translation.y,
                             yaw_of(tr.transform.rotation))
                        if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                            m2o = p
                        elif tr.header.frame_id == 'odom':
                            o2b = p
                else:
                    latest[k] = m
                i += 1

            ax.clear()

            # The map.
            if '/map' in latest:
                img, ext = grid_image(latest['/map'])
                ax.imshow(img, origin='lower', extent=ext, cmap='gray', vmin=0, vmax=1)

            # The estimated path.
            if '/slam_path' in latest and latest['/slam_path'].poses:
                pts = latest['/slam_path'].poses
                ax.plot([p.pose.position.x for p in pts], [p.pose.position.y for p in pts],
                        'r-', lw=1.5, label='estimated path')

            # The particle cloud: one arrow per particle.
            if '/particles' in latest:
                ps = latest['/particles'].poses
                px = np.array([p.position.x for p in ps])
                py = np.array([p.position.y for p in ps])
                pth = np.array([yaw_of(p.orientation) for p in ps])
                ax.quiver(px, py, np.cos(pth), np.sin(pth), color='tab:blue', scale=25,
                          width=0.004, label='particles')

            # The live scan at the live estimated pose.
            if '/scan' in latest and o2b is not None:
                c, s_ = math.cos(m2o[2]), math.sin(m2o[2])
                live = (m2o[0] + c * o2b[0] - s_ * o2b[1],
                        m2o[1] + s_ * o2b[0] + c * o2b[1],
                        m2o[2] + o2b[2])
                sx, sy = scan_points(latest['/scan'], live)
                ax.plot(sx, sy, '.', color='tab:green', ms=2, label='laser scan')

            ax.set_xlim(cx - half, cx + half)
            ax.set_ylim(cy - half, cy + half)
            ax.set_aspect('equal')
            ax.set_xlabel('x [m]')
            ax.set_ylabel('y [m]')
            ax.set_title('%s   t = %5.1f s   (%gx speed)' % (title, tf - t0, a.speed))
            ax.legend(loc='upper right', fontsize=8)

            writer.grab_frame()

    print('wrote', a.out, '(%d frames)' % len(frames))


if __name__ == '__main__':
    main()
