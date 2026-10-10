#!/usr/bin/env python3
"""Plot the robot's trajectory, and what the lidar saw, from a bag.

Usage (ROS environment must be sourced first, for rosbag2_py and the message types):

    source /opt/ros/jazzy/setup.bash
    source <workspace>/install/setup.bash
    python3 scripts/plot_trajectory.py bags/task1-position -o task1_trajectory.png

Three layers, any of which may be absent from a given bag:

- every valid /scan beam, transformed into the odometry frame with the pose the robot
  held when that scan arrived. Accumulated over a run these draw the room, which is
  what makes a wall following plot readable: the path alone shows a loop, the hits
  show what it was a loop *around*.
- /odom, the path itself, with the start and end marked.
- /new_position, the commanded goal(s) -- a scatter for a handful of discrete
  setpoints (task 1), a line for a continuously published trajectory (task 2).

The summary box carries the numbers the report needs: how far the robot drove, how
long it took, and for a closed loop how far the end landed from the start, which is
task 4's acceptance number.
"""

import argparse
import math
import pathlib

import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

# base_link -> base_scan on the Burger, from the URDF. The lidar sits 3.2 cm behind
# the wheel axle, so without this every hit is drawn 3.2 cm out along the heading and
# the walls come out slightly thickened on the turns.
BURGER_SCAN_OFFSET_X = -0.032


def open_reader(bag_path: str):
    """Open a bag, trying each storage plugin rather than assuming mcap."""
    last = None
    for storage_id in ('mcap', 'sqlite3', ''):
        try:
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id),
                rosbag2_py.ConverterOptions('', ''),
            )
            return reader
        except Exception as exc:                                  # noqa: BLE001
            last = exc
    raise SystemExit('could not open %s as a rosbag2 bag: %s' % (bag_path, last))


def read_topic(bag_path: str, topic: str):
    """Yield (timestamp_seconds, message) for every message on `topic`."""
    reader = open_reader(bag_path)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        return
    msg_type = get_message(types[topic])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    while reader.has_next():
        _, data, stamp = reader.read_next()
        yield stamp * 1e-9, deserialize_message(data, msg_type)


def yaw_of(q) -> float:
    """Heading from a quaternion, the z rotation only."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_path(bag_path: str, topic: str):
    """Return (t, x, y, yaw) arrays from an Odometry topic."""
    t, x, y, yaw = [], [], [], []
    for stamp, msg in read_topic(bag_path, topic):
        t.append(stamp)
        x.append(msg.pose.pose.position.x)
        y.append(msg.pose.pose.position.y)
        yaw.append(yaw_of(msg.pose.pose.orientation))
    return (np.array(t), np.array(x), np.array(y), np.array(yaw))


def read_goals(bag_path: str, topic: str):
    """Return (x, y) arrays from a Pose topic."""
    x, y = [], []
    for _, msg in read_topic(bag_path, topic):
        position = msg.pose.pose.position if hasattr(msg, 'pose') else msg.position
        x.append(position.x)
        y.append(position.y)
    return np.array(x), np.array(y)


def pose_at(path, stamps):
    """Interpolate the robot pose onto `stamps`.

    Odometry runs far faster than the lidar, so this is a short interpolation rather
    than an extrapolation. Heading is interpolated through its sine and cosine, since
    averaging raw angles across the +/-pi wrap gives a heading pointing the other way.
    """
    t, x, y, yaw = path
    px = np.interp(stamps, t, x)
    py = np.interp(stamps, t, y)
    pyaw = np.arctan2(np.interp(stamps, t, np.sin(yaw)),
                      np.interp(stamps, t, np.cos(yaw)))
    return px, py, pyaw


def scan_hits(bag_path: str, topic: str, path, offset_x: float, max_points: int):
    """Every valid scan return, in the odometry frame.

    A beam is kept only if it is finite and inside the scanner's own declared range:
    an LDS-01 reports 0.0 or inf for a beam that hit nothing, and plotting those puts
    a spray of points on the robot and at the horizon.
    """
    stamps, bearings, ranges = [], [], []
    for stamp, msg in read_topic(bag_path, topic):
        values = np.asarray(msg.ranges, dtype=float)
        if values.size == 0:
            continue
        angles = msg.angle_min + np.arange(values.size) * msg.angle_increment
        good = (np.isfinite(values) & (values >= msg.range_min)
                & (values <= msg.range_max))
        if not good.any():
            continue
        stamps.append(np.full(int(good.sum()), stamp))
        bearings.append(angles[good])
        ranges.append(values[good])
    if not stamps:
        return np.array([]), np.array([])

    stamps = np.concatenate(stamps)
    bearings = np.concatenate(bearings)
    ranges = np.concatenate(ranges)

    if path[0].size == 0:
        return np.array([]), np.array([])
    px, py, pyaw = pose_at(path, stamps)
    world = pyaw + bearings
    hx = px + offset_x * np.cos(pyaw) + ranges * np.cos(world)
    hy = py + offset_x * np.sin(pyaw) + ranges * np.sin(world)

    # Thin evenly rather than by truncating, so a long run still draws the whole room.
    if max_points and hx.size > max_points:
        keep = np.linspace(0, hx.size - 1, max_points).astype(int)
        hx, hy = hx[keep], hy[keep]
    return hx, hy


def lap_return(x, y, min_lap: float):
    """Find the earlier pass the end of the run comes closest to.

    Returns (index, distance, lap_length), or None if the run never came back -- a
    there-and-stop run (tasks 1 and 2) has no lap and should not be annotated with one.

    Only points at least `min_lap` of travel back down the path are eligible, or the
    answer is always the previous sample.
    """
    if x.size < 3:
        return None
    travel = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    eligible = np.flatnonzero(travel[-1] - travel >= min_lap)
    if eligible.size == 0:
        return None
    distances = np.hypot(x[eligible] - x[-1], y[eligible] - y[-1])
    best = int(eligible[np.argmin(distances)])
    error = float(distances.min())
    # A run that merely stopped somewhere far from its own track has not closed a loop.
    if error > 0.15 * travel[-1]:
        return None
    return best, error, float(travel[-1] - travel[best])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='path to the bag directory')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--goal-topic', default='/new_position')
    parser.add_argument('--scan-topic', default='/scan')
    parser.add_argument('--no-scan', action='store_true',
                        help='skip the lidar layer (faster, and what tasks 1 and 2 '
                             'usually want)')
    parser.add_argument('--max-points', type=int, default=150000,
                        help='thin the lidar hits to at most this many points')
    parser.add_argument('--scan-offset-x', type=float, default=BURGER_SCAN_OFFSET_X,
                        help='lidar x offset from the odometry frame, metres')
    parser.add_argument('--min-lap', type=float, default=2.0,
                        help='how far back down the path to look for the lap '
                             'reference, metres; matches loop_min_distance')
    parser.add_argument('-t', '--title', default=None)
    parser.add_argument('-o', '--out', default=None,
                        help='output image path, defaults to <bag name>_trajectory.png')
    args = parser.parse_args()

    path = read_path(args.bag, args.odom_topic)
    if path[0].size == 0:
        raise SystemExit('no messages found on %s in %s' % (args.odom_topic, args.bag))
    _, rx, ry, _ = path

    fig, ax = plt.subplots(figsize=(7.5, 7.5))

    if not args.no_scan:
        hx, hy = scan_hits(args.bag, args.scan_topic, path,
                           args.scan_offset_x, args.max_points)
        if hx.size:
            ax.scatter(hx, hy, s=0.5, c='#2B2B2B', alpha=0.25, linewidths=0,
                       label='lidar hits (/scan)', rasterized=True)

    ax.plot(rx, ry, '-', color='#1547D6', linewidth=1.4, label='robot path (/odom)')
    ax.plot(rx[0], ry[0], 'o', color='#1D9E75', markersize=9,
            markeredgecolor='white', label='start')
    ax.plot(rx[-1], ry[-1], 'o', color='#D62F2F', markersize=9,
            markeredgecolor='white', label='end')

    goal_x, goal_y = read_goals(args.bag, args.goal_topic)
    if goal_x.size:
        # A terminal goal republished at 5 Hz for a few seconds is still one goal, not
        # a path, so count distinct setpoints rather than messages.
        distinct = {(round(x, 3), round(y, 3)) for x, y in zip(goal_x, goal_y)}
        if len(distinct) > 20:
            ax.plot(goal_x, goal_y, '--', color='#D85A30', linewidth=1.5,
                    label='provided trajectory (/new_position)')
        else:
            dx, dy = zip(*distinct)
            ax.plot(dx, dy, 'x', color='#D85A30', markersize=10, markeredgewidth=2,
                    linestyle='none', label='commanded goal(s) (/new_position)')

    travelled = float(np.hypot(np.diff(rx), np.diff(ry)).sum())
    duration = float(path[0][-1] - path[0][0])
    lines = ['path %.2f m over %.0f s' % (travelled, duration)]

    # Task 4's number is how close the robot came back to the track it left, which is
    # not the same as how close it came back to wherever the recording happened to
    # start: the lap is referenced to the point the follower settled onto the wall,
    # some way into the run. Recovering it from the bag alone means finding the
    # earlier pass the end of the run comes closest to, which needs no extra topic and
    # works whether or not the recorder was running when the robot was switched on.
    closed = lap_return(rx, ry, args.min_lap)
    if closed is not None:
        index, error, lap_length = closed
        lines.append('closest return to its own track: %.3f m' % error)
        lines.append('after %.2f m of lap' % lap_length)
        ax.plot([rx[index], rx[-1]], [ry[index], ry[-1]], ':', color='#D62F2F',
                linewidth=1.4)
        ax.plot(rx[index], ry[index], 's', color='#D62F2F', markersize=7,
                markerfacecolor='none', markeredgewidth=1.6, label='lap reference')

    ax.text(0.02, 0.02, '\n'.join(lines), transform=ax.transAxes, fontsize=9,
            va='bottom', ha='left',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='white', alpha=0.8,
                      edgecolor='#BBBBBB'))

    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', framealpha=0.9)
    ax.set_title(args.title or pathlib.Path(args.bag).name)

    out = args.out or '%s_trajectory.png' % pathlib.Path(args.bag).name
    fig.savefig(out, dpi=150, bbox_inches='tight')
    print('saved %s' % out)


if __name__ == '__main__':
    main()
