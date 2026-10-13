#!/usr/bin/env python3
"""Create report figures from a Lab 3 bag, without replaying ROS.

    source /opt/ros/jazzy/setup.bash
    source ~/R7021E-group6/lab3/ros2_ws/install/setup.bash
    python3 ~/R7021E-group6/lab3/video/bag_to_report.py ~/bags/hall-1

By default, writes <bag-name>_report/ beside the bag. The map figure shows the
final SLAM map and planner layers, plus the robot trail in map coordinates.
The metrics figure shows known mapped area (not a completion percentage) and
recorded velocity commands against elapsed bag receive time.
"""

import argparse
import pathlib

from bag_to_video import COLORS, grid_image, read_bag

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

import numpy as np  # noqa: E402


def same_grid(a, b):
    """Whether two OccupancyGrids describe the same axis-aligned cells."""
    if a.info.width != b.info.width or a.info.height != b.info.height:
        return False
    if abs(a.info.resolution - b.info.resolution) > 1e-9:
        return False
    ax, ay = a.info.origin.position.x, a.info.origin.position.y
    bx, by = b.info.origin.position.x, b.info.origin.position.y
    return abs(ax - bx) <= 1e-6 and abs(ay - by) <= 1e-6


def collect_run(events, poses):
    """Keep final map layers and plan, plus area and velocity time series."""
    latest = {}
    known_time, known_area = [], []
    command_time, linear_x, angular_z = [], [], []

    for stamp, topic, msg in events:
        if topic == '/map':
            latest['/map'] = msg
            values = np.asarray(msg.data, dtype=np.int16)
            known_time.append(stamp)
            known_area.append(float(
                np.count_nonzero(values >= 0) * msg.info.resolution ** 2))
        elif topic == '/inflated_map':
            latest[topic] = msg
        elif topic == '/frontiers':
            latest[topic] = msg
        elif topic == '/path' and len(msg.poses) > 1:
            latest[topic] = msg
        elif topic == '/rrt_tree' and len(msg.points) >= 2:
            latest[topic] = msg
        elif topic == '/goal_marker' and msg.action == msg.ADD:
            latest[topic] = msg
        elif topic == '/cmd_vel':
            command_time.append(stamp)
            linear_x.append(msg.twist.linear.x)
            angular_z.append(msg.twist.angular.z)

    if '/map' not in latest:
        raise SystemExit(
            'No /map messages found. Check that /map was recorded.')
    if poses.size == 0:
        raise SystemExit(
            'No usable /tf robot poses found. Check that /tf was recorded.')
    if not known_time:
        raise SystemExit('No map area samples were decoded from the bag.')

    start = min(known_time[0], poses[0, 0],
                command_time[0] if command_time else known_time[0])
    return {
        'latest': latest,
        'poses': poses,
        'known_time': np.asarray(known_time) - start,
        'known_area': np.asarray(known_area),
        'command_time': np.asarray(command_time) - start,
        'linear_x': np.asarray(linear_x),
        'angular_z': np.asarray(angular_z),
        'duration': (max(known_time[-1], poses[-1, 0],
                         command_time[-1] if command_time else known_time[-1])
                     - start),
    }


def plot_map(run, title, out):
    """Save final occupancy with truthful planning and trajectory layers."""
    latest = run['latest']
    msg = latest['/map']
    grid, extent = grid_image(msg)
    image = np.empty(grid.shape + (3,), dtype=float)
    image[:] = matplotlib.colors.to_rgb(COLORS['unknown'])
    image[(grid >= 0) & (grid <= 50)] = matplotlib.colors.to_rgb(
        COLORS['free'])

    safe = latest.get('/inflated_map')
    if safe is not None:
        if same_grid(msg, safe):
            safe_grid = np.asarray(safe.data, dtype=np.int16).reshape(
                grid.shape)
            image[safe_grid > 50] = matplotlib.colors.to_rgb(COLORS['safe'])
        else:
            print('warning: /inflated_map grid differs from /map; '
                  'omitting that overlay')

    image[grid > 50] = matplotlib.colors.to_rgb(COLORS['occupied'])
    frontiers = latest.get('/frontiers')
    if frontiers is not None:
        if same_grid(msg, frontiers):
            frontier_grid = np.asarray(frontiers.data, dtype=np.int16).reshape(
                grid.shape)
            image[frontier_grid > 50] = matplotlib.colors.to_rgb(
                COLORS['frontier'])
        else:
            print('warning: /frontiers grid differs from /map; '
                  'omitting that overlay')

    fig, ax = plt.subplots(figsize=(9, 8))
    ax.imshow(image, origin='lower', extent=extent, interpolation='nearest')

    tree = latest.get('/rrt_tree')
    if tree is not None:
        points = np.asarray([(p.x, p.y) for p in tree.points], dtype=float)
        if points.size >= 4:
            segments = points[:(len(points) // 2) * 2].reshape(-1, 2, 2)
            ax.add_collection(LineCollection(segments, colors=COLORS['tree'],
                                             linewidths=0.45, alpha=0.75))

    path = latest.get('/path')
    if path is not None:
        points = np.asarray([(p.pose.position.x, p.pose.position.y)
                             for p in path.poses])
        ax.plot(points[:, 0], points[:, 1], color=COLORS['path'],
                linewidth=2.0)

    poses = run['poses']
    ax.plot(poses[:, 1], poses[:, 2], color=COLORS['trail'], linewidth=1.5)
    ax.scatter(poses[0, 1], poses[0, 2], color='#1d9e75', edgecolor='white',
               linewidth=0.8, s=75, zorder=5)
    ax.scatter(poses[-1, 1], poses[-1, 2], color=COLORS['trail'], marker='X',
               edgecolor='white', linewidth=0.8, s=85, zorder=5)
    goal = latest.get('/goal_marker')
    if goal is not None:
        ax.scatter(goal.pose.position.x, goal.pose.position.y,
                   color=COLORS['goal'], marker='o', edgecolor='white',
                   linewidth=0.8, s=75, zorder=6)

    x_min = min(extent[0], float(poses[:, 1].min()))
    x_max = max(extent[1], float(poses[:, 1].max()))
    y_min = min(extent[2], float(poses[:, 2].min()))
    y_max = max(extent[3], float(poses[:, 2].max()))
    pad = 0.15
    ax.set_xlim(x_min - pad, x_max + pad)
    ax.set_ylim(y_min - pad, y_max + pad)
    ax.set_aspect('equal')
    ax.set_xlabel('x in map frame [m]')
    ax.set_ylabel('y in map frame [m]')
    ax.grid(True, alpha=0.2)
    ax.set_title(f'{title}: final SLAM map and navigation evidence')

    handles = [
        Patch(facecolor=COLORS['unknown'], label='unknown'),
        Patch(facecolor=COLORS['free'], edgecolor='#999999',
              label='known free'),
        Patch(facecolor=COLORS['occupied'], label='occupied'),
        Patch(facecolor=COLORS['safe'], label='planner-safe (/inflated_map)'),
        Patch(facecolor=COLORS['frontier'], label='frontier cells'),
        Line2D([], [], color=COLORS['tree'], lw=1, label='last RRT* tree'),
        Line2D([], [], color=COLORS['path'], lw=2, label='last planned path'),
        Line2D([], [], color=COLORS['trail'], lw=1.5,
               label='robot trajectory (/tf)'),
        Line2D([], [], color='#1d9e75', marker='o', ls='none', label='start'),
        Line2D([], [], color=COLORS['trail'], marker='X', ls='none',
               label='recorded end'),
        Line2D([], [], color=COLORS['goal'], marker='o', ls='none',
               label='last planned goal'),
    ]
    ax.legend(handles=handles, loc='upper center', bbox_to_anchor=(0.5, -0.10),
              ncol=3, fontsize=8, framealpha=0.95)
    fig.subplots_adjust(bottom=0.23, top=0.91)
    fig.savefig(out, dpi=180, bbox_inches='tight')
    plt.close(fig)


def plot_metrics(run, title, out):
    """Save mapped area and commands against elapsed bag receive time."""
    fig, (area_ax, speed_ax) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    area_ax.plot(run['known_time'], run['known_area'], color='#1547d6',
                 linewidth=1.8, marker='.', markersize=3)
    area_ax.set_ylabel('known map area [m²]')
    area_ax.set_title(f'{title}: exploration and command history')
    area_ax.grid(True, alpha=0.3)

    handles = [Line2D([], [], color='#1547d6', lw=1.8,
                      label='known cells × map resolution²')]
    if run['command_time'].size:
        speed_ax.plot(run['command_time'], run['linear_x'], color='#008040',
                      linewidth=1.0, label='linear x')
        speed_ax.set_ylabel('linear command [m/s]', color='#008040')
        speed_ax.tick_params(axis='y', labelcolor='#008040')
        angular_ax = speed_ax.twinx()
        angular_ax.plot(run['command_time'], run['angular_z'], color='#d85a30',
                        linewidth=1.0, alpha=0.9, label='angular z')
        angular_ax.set_ylabel('angular command [rad/s]', color='#d85a30')
        angular_ax.tick_params(axis='y', labelcolor='#d85a30')
        handles.extend([
            Line2D([], [], color='#008040', lw=1, label='/cmd_vel linear x'),
            Line2D([], [], color='#d85a30', lw=1, label='/cmd_vel angular z'),
        ])
    else:
        speed_ax.text(0.5, 0.5, 'No /cmd_vel samples in this bag',
                      transform=speed_ax.transAxes, ha='center', va='center')
        speed_ax.set_ylabel('velocity command')

    speed_ax.set_xlabel('elapsed time from bag receive timestamps [s]')
    speed_ax.grid(True, alpha=0.3)
    fig.legend(handles=handles, loc='lower center', ncol=len(handles),
               bbox_to_anchor=(0.5, 0.01), framealpha=0.95)
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(out, dpi=180, bbox_inches='tight')
    plt.close(fig)


def main():
    """Generate both Lab 3 report figures."""
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bag', help='path to a ROS 2 bag directory')
    parser.add_argument('--out-dir', default=None,
                        help='output folder; defaults to <bag-name>_report '
                             'beside the bag')
    args = parser.parse_args()

    bag = pathlib.Path(args.bag).expanduser().resolve()
    run_name = bag.parent.name if bag.name == 'bag' else bag.name
    if args.out_dir:
        out_dir = pathlib.Path(args.out_dir).expanduser().resolve()
    else:
        out_dir = bag.parent / f'{run_name}_report'
    out_dir.mkdir(parents=True, exist_ok=True)

    events, poses = read_bag(str(bag))
    run = collect_run(events, poses)
    map_out = out_dir / f'{run_name}_map.png'
    metrics_out = out_dir / f'{run_name}_metrics.png'
    plot_map(run, run_name, map_out)
    plot_metrics(run, run_name, metrics_out)

    distance = float(
        np.hypot(np.diff(poses[:, 1]), np.diff(poses[:, 2])).sum())
    print(f'bag duration represented: {run["duration"]:.1f} s')
    print(f'final known map area: {run["known_area"][-1]:.2f} m^2 '
          '(not a completion percentage; no ground-truth map is in the bag)')
    print(f'recorded trajectory length: {distance:.2f} m')
    print(f'wrote {map_out}')
    print(f'wrote {metrics_out}')


if __name__ == '__main__':
    main()
