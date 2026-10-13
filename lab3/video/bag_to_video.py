#!/usr/bin/env python3
"""Make a demo video of a lab 3 exploration run from its bag. No ROS graph, no RViz.

    source /opt/ros/jazzy/setup.bash
    python3 ~/R7021E-group6/lab3/video/bag_to_video.py ~/bags/<run> [--speed 10] [--out run.mp4]

Each frame shows the SLAM map, the planner-blocked wall/inflation cells (/inflated_map),
frontiers, the most recent RRT* tree and path, the most recent goal, and the robot's recorded
trail. A legend is included in every frame. The robot pose is map->odom->base_footprint from /tf.

Time comes from the bag (when the laptop received each message), not from message stamps.
The robot clock can be far from the laptop clock, and this way that does not matter.
"""

import argparse
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FFMpegWriter, PillowWriter  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402
import numpy as np  # noqa: E402
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from geometry_msgs.msg import TwistStamped  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Path  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402

TYPES = {'/map': OccupancyGrid, '/inflated_map': OccupancyGrid, '/frontiers': OccupancyGrid,
         '/path': Path, '/rrt_tree': Marker, '/goal_marker': Marker,
         '/cmd_vel': TwistStamped}

COLORS = {
    'unknown': '#cccccc',
    'free': '#ffffff',
    'occupied': '#252525',
    'inflated': '#f2b6b6',
    'frontier': '#1ebf32',
    'tree': '#33b3b3',
    'path': '#008040',
    'goal': '#d62728',
    'trail': '#1547d6',
    'robot': '#1f77b4',
}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read_bag(uri):
    """Return a time-sorted list of (t, topic, message) and the robot poses [(t, x, y, yaw)]."""
    reader = rosbag2_py.SequentialReader()
    storage = 'mcap' if any(f.endswith('.mcap') for f in os.listdir(uri)) else 'sqlite3'
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=storage),
                rosbag2_py.ConverterOptions('', ''))
    events, poses = [], []
    m2o, o2b = (0.0, 0.0, 0.0), None
    while reader.has_next():
        topic, data, t = reader.read_next()
        t *= 1e-9
        if topic == '/tf':
            for tf in deserialize_message(data, TFMessage).transforms:
                p = (tf.transform.translation.x, tf.transform.translation.y,
                     yaw_of(tf.transform.rotation))
                if tf.header.frame_id == 'map' and tf.child_frame_id == 'odom':
                    m2o = p
                elif tf.header.frame_id == 'odom':
                    o2b = p
            if o2b is not None:
                x = m2o[0] + math.cos(m2o[2]) * o2b[0] - math.sin(m2o[2]) * o2b[1]
                y = m2o[1] + math.sin(m2o[2]) * o2b[0] + math.cos(m2o[2]) * o2b[1]
                poses.append((t, x, y, m2o[2] + o2b[2]))
        elif topic in TYPES:
            events.append((t, topic, deserialize_message(data, TYPES[topic])))
    return events, np.array(poses)


def grid_image(msg):
    g = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
    o, r = msg.info.origin.position, msg.info.resolution
    return g, [o.x, o.x + g.shape[1] * r, o.y, o.y + g.shape[0] * r]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--speed', type=float, default=10.0, help='times faster than real time')
    ap.add_argument('--fps', type=int, default=10)
    ap.add_argument('--out', default=None, help='.mp4 (needs ffmpeg) or .gif')
    a = ap.parse_args()
    if a.speed <= 0 or a.fps <= 0:
        ap.error('--speed and --fps must be positive')
    bag = os.path.expanduser(a.bag).rstrip('/')
    out = a.out or os.path.basename(bag) + '.mp4'
    if not out.lower().endswith(('.mp4', '.gif')):
        ap.error('--out must end in .mp4 or .gif')

    events, poses = read_bag(bag)
    if len(poses) == 0 or not any(e[1] == '/map' for e in events):
        raise SystemExit('No /tf robot pose or no /map in this bag. Check the record line.')
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    t0, t1 = poses[0, 0], poses[-1, 0]
    step = a.speed / a.fps                       # bag seconds per video frame
    frames = np.arange(t0, t1, step)
    print(f'{len(events)} messages, {t1 - t0:.0f} s of bag, {len(frames)} frames -> {out}')

    # The view covers every map the run produced, so the axes do not jump.
    ext = np.array([grid_image(m)[1] for _, k, m in events if k == '/map'])
    view = [ext[:, 0].min(), ext[:, 1].max(), ext[:, 2].min(), ext[:, 3].max()]

    fig, ax = plt.subplots(figsize=(8, 8))
    fig.subplots_adjust(top=0.90, bottom=0.23)
    writer = FFMpegWriter(fps=a.fps) if out.lower().endswith('.mp4') else PillowWriter(fps=a.fps)
    latest, i = {}, 0
    legend = [
        Patch(facecolor=COLORS['unknown'], label='unknown'),
        Patch(facecolor=COLORS['free'], edgecolor='#999999', label='known free'),
        Patch(facecolor=COLORS['occupied'], label='occupied'),
        Patch(facecolor=COLORS['inflated'],
              label='inflated/blocked cells (/inflated_map)'),
        Patch(facecolor=COLORS['frontier'], label='frontier cells'),
        Line2D([], [], color=COLORS['tree'], lw=1, label='last RRT* tree'),
        Line2D([], [], color=COLORS['path'], lw=2, label='last planned path'),
        Line2D([], [], color=COLORS['goal'], marker='o', ls='none', label='last goal'),
        Line2D([], [], color=COLORS['trail'], lw=1.5, label='robot trail'),
        Line2D([], [], color=COLORS['robot'], marker='o', ls='none',
               label='robot position and heading'),
    ]
    with writer.saving(fig, out, dpi=100):
        for tf in frames:
            while i < len(events) and events[i][0] <= tf:
                latest[events[i][1]] = events[i][2]
                i += 1
            if '/map' not in latest:
                continue
            ax.clear()
            g, e = grid_image(latest['/map'])
            img = np.empty(g.shape + (3,), dtype=float)
            img[:] = matplotlib.colors.to_rgb(COLORS['unknown'])
            img[(g >= 0) & (g <= 50)] = matplotlib.colors.to_rgb(COLORS['free'])
            if '/inflated_map' in latest:
                ig, ie = grid_image(latest['/inflated_map'])
                if (ig.shape == g.shape
                        and np.allclose(ie, e, atol=1e-6)
                        and abs(latest['/inflated_map'].info.resolution
                                - latest['/map'].info.resolution) < 1e-9):
                    blocked = (ig > 50) & (g <= 50)
                    img[blocked] = matplotlib.colors.to_rgb(COLORS['inflated'])
            img[g > 50] = matplotlib.colors.to_rgb(COLORS['occupied'])
            if '/frontiers' in latest:
                fg, fe = grid_image(latest['/frontiers'])
                if (fg.shape == g.shape and np.allclose(fe, e, atol=1e-6)
                        and abs(latest['/frontiers'].info.resolution
                                - latest['/map'].info.resolution) < 1e-9):
                    img[fg > 50] = matplotlib.colors.to_rgb(COLORS['frontier'])
            ax.imshow(img, origin='lower', extent=e, interpolation='nearest')
            if '/rrt_tree' in latest:
                pts = np.array([(p.x, p.y) for p in latest['/rrt_tree'].points])
                if len(pts):
                    seg = pts.reshape(-1, 2, 2)
                    for s in seg:
                        ax.plot(s[:, 0], s[:, 1], color=COLORS['tree'], lw=0.3)
            if '/path' in latest and len(latest['/path'].poses) > 1:
                p = np.array([(q.pose.position.x, q.pose.position.y)
                              for q in latest['/path'].poses])
                ax.plot(p[:, 0], p[:, 1], color=COLORS['path'], lw=2)
            if '/goal_marker' in latest:
                gm = latest['/goal_marker'].pose.position
                ax.plot(gm.x, gm.y, 'o', color=COLORS['goal'], ms=10)
            k = np.searchsorted(poses[:, 0], tf)
            trail = poses[:max(k, 1)]
            ax.plot(trail[:, 1], trail[:, 2], color=COLORS['trail'], lw=1)
            x, y, th = trail[-1, 1:]
            ax.add_patch(plt.Circle((x, y), 0.105, color=COLORS['robot'], alpha=0.4))
            ax.arrow(x, y, 0.15 * math.cos(th), 0.15 * math.sin(th), width=0.01,
                     color=COLORS['robot'])
            ax.set_xlim(view[0], view[1])
            ax.set_ylim(view[2], view[3])
            ax.set_aspect('equal')
            ax.set_title(f'{os.path.basename(bag)}   t = {tf - t0:5.0f} s   ({a.speed:g}x)')
            ax.legend(handles=legend, loc='upper center', bbox_to_anchor=(0.5, -0.08),
                      ncol=3, fontsize=7, framealpha=0.95)
            writer.grab_frame()
    print('wrote', out)


if __name__ == '__main__':
    main()
