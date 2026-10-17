#!/usr/bin/env python3
"""All Lab 1 report figures, from the bags in the submission folder.

    source /opt/ros/jazzy/setup.bash
    python3 ~/R7021E-group6/lab1/plots/make_figures.py [--bags <folder with robot-2026-09-30/ and simulation/>]

One style and one set of legend labels for every figure.
"""

import argparse
import math
import pathlib

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
import rosbag2_py  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
L, KP = 0.10, 0.8                      # NID offset and gain on the day
BLUE, ORANGE, AQUA, GREY, INK = '#2a78d6', '#eb6834', '#1baf7a', '#9a998f', '#1f1f1e'
LABEL = dict(path='robot path (/odom)', p='NID point P', goal='goal (/new_position)',
             ref='reference (/new_position)', scan='lidar hits (/scan)', start='start', end='end')
plt.rcParams.update({'font.size': 10, 'axes.grid': True, 'grid.color': '#e4e3dd',
                     'axes.spines.top': False, 'axes.spines.right': False, 'legend.frameon': False,
                     'savefig.bbox': 'tight', 'savefig.dpi': 150})


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def load(bag):
    """odom (t, x, y, th), cmd (t, v, w), goals (t, x, y), scans (t, angle_min, inc, ranges, rmin, rmax)."""
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    odom, cmd, goal, scan, t0 = [], [], [], [], None
    while r.has_next():
        topic, data, t = r.read_next()
        t = t * 1e-9
        t0 = t if t0 is None else t0
        if topic not in ('/odom', '/cmd_vel', '/new_position', '/scan'):
            continue
        m = deserialize_message(data, types[topic])
        if topic == '/odom':
            p = m.pose.pose
            odom.append((t - t0, p.position.x, p.position.y, yaw(p.orientation)))
        elif topic == '/cmd_vel':
            cmd.append((t - t0, m.twist.linear.x, m.twist.angular.z))
        elif topic == '/new_position':
            goal.append((t - t0, m.position.x, m.position.y))
        else:
            scan.append((t - t0, m.angle_min, m.angle_increment, np.array(m.ranges), m.range_min, m.range_max))
    arr = lambda a, n: np.array(a) if a else np.zeros((0, n))
    return arr(odom, 4), arr(cmd, 3), arr(goal, 3), scan


def scan_hits(odom, scans, every=5):
    xs, ys = [], []
    for t, a0, da, r, rmin, rmax in scans[::every]:
        i = min(np.searchsorted(odom[:, 0], t), len(odom) - 1)
        x, y, th = odom[i, 1:4]
        ok = (r > rmin) & (r < rmax)
        a = th + a0 + da * np.arange(len(r))
        xs.append(x - 0.032 * math.cos(th) + r[ok] * np.cos(a[ok]))
        ys.append(y - 0.032 * math.sin(th) + r[ok] * np.sin(a[ok]))
    return (np.concatenate(xs), np.concatenate(ys)) if xs else (np.array([]), np.array([]))


def p_point(odom):
    return odom[:, 1] + L * np.cos(odom[:, 3]), odom[:, 2] + L * np.sin(odom[:, 3])


def start_end(ax, odom):
    ax.plot(odom[0, 1], odom[0, 2], 'o', color=INK, ms=8, label=LABEL['start'])
    ax.plot(odom[-1, 1], odom[-1, 2], 's', color=INK, ms=7, mfc='white', label=LABEL['end'])


def finish(ax, fig, title, name, legend='best'):
    ax.set(xlabel='x [m]', ylabel='y [m]', aspect='equal', title=title)
    if legend == 'below':
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=3, fontsize=8.5)
    else:
        ax.legend(loc=legend, fontsize=8.5)
    fig.savefig(HERE / name)
    plt.close(fig)
    print('saved', HERE / name)


def goal_at(goals, t):
    return np.interp(t, goals[:, 0], goals[:, 1]), np.interp(t, goals[:, 0], goals[:, 2])


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--bags', default=str(pathlib.Path.home() / 'Downloads/R7021E-lab1-submission/bags'))
    parser.add_argument('--desk-block', default=None, help='optional .npy path from the desk block run')
    args = parser.parse_args()
    robot, sim = pathlib.Path(args.bags) / 'robot-2026-09-30', pathlib.Path(args.bags) / 'simulation'

    # Task 1: goals, robot path, NID point.
    o, c, g, _ = load(robot / 'task1-position-1m-forward')
    px, py = p_point(o)
    goals = {(1.0, 0.0)} | {(round(x, 3), round(y, 3)) for x, y in g[:, 1:]}  # (1, 0) was sent before recording
    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    ax.plot(o[:, 1], o[:, 2], color=BLUE, label=LABEL['path'])
    ax.plot(px, py, color=ORANGE, ls='--', lw=1.4, label=LABEL['p'])
    gx, gy = zip(*sorted(goals))
    ax.plot(gx, gy, 'x', color=INK, ms=10, mew=2, ls='none', label=LABEL['goal'])
    start_end(ax, o)
    finish(ax, fig, 'Lab 1 Task 1', 'lab1_task1.png', 'center')

    # Task 2: reference, robot path, NID point.
    o, c, g, _ = load(robot / 'task2-draw-eight')
    px, py = p_point(o)
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(g[:, 1], g[:, 2], ':', color=GREY, lw=2, label=LABEL['ref'])
    ax.plot(px, py, color=ORANGE, ls='--', lw=1.4, label=LABEL['p'])
    ax.plot(o[:, 1], o[:, 2], color=BLUE, lw=1.6, label=LABEL['path'])
    start_end(ax, o)
    finish(ax, fig, 'Lab 1 Task 2', 'lab1_task2.png', 'below')

    # Task 2: tracking error over time, with the odometry gaps.
    m = (o[:, 0] > g[0, 0]) & (o[:, 0] < g[-1, 0])
    GX, GY = goal_at(g, o[m, 0])
    fig, ax = plt.subplots(figsize=(9, 3.6))
    ax.plot(o[m, 0], np.hypot(GX - px[m], GY - py[m]), color=ORANGE, ls='--', label='NID point P to reference')
    ax.plot(o[m, 0], np.hypot(GX - o[m, 1], GY - o[m, 2]), color=BLUE, label='robot centre to reference')
    for i in np.flatnonzero(np.diff(o[:, 0]) > 0.5):
        ax.axvspan(o[i, 0], o[i + 1, 0], color=GREY, alpha=0.35, lw=0)
    ax.axvspan(0, 0, color=GREY, alpha=0.35, label='no odometry for over 0.5 s')
    ax.set(xlabel='time [s]', ylabel='error [m]', title='Lab 1 Task 2, tracking error')
    ax.legend(fontsize=8.5, ncol=3, loc='upper left')
    fig.savefig(HERE / 'lab1_task2_error.png'); plt.close(fig); print('saved', HERE / 'lab1_task2_error.png')

    # Task 2: the eight is drawn late, not wrong. Error against a delayed reference.
    sel = (o[:, 0] > g[0, 0] + 6) & (o[:, 0] < g[-1, 0] - 2)
    taus = np.arange(0, 3.01, 0.05)
    err = [np.hypot(*(np.array(goal_at(g, o[sel, 0] - t)) - np.array([px[sel], py[sel]]))).mean() for t in taus]
    k = int(np.argmin(err))
    fig, ax = plt.subplots(figsize=(7, 3.8))
    ax.plot(taus, err, color=BLUE)
    ax.plot(taus[k], err[k], 'o', color=BLUE, ms=8)
    ax.annotate('best fit %.2f s' % taus[k], (taus[k], err[k]), xytext=(8, 8), textcoords='offset points')
    ax.axvline(1 / KP, color=GREY, ls=':')
    ax.text(1 / KP, max(err) * 0.95, ' 1/k_p = 1.25 s', color=GREY, va='top')
    ax.set(xlabel='delay applied to the reference [s]', ylabel='mean error of P [m]', title='Lab 1 Task 2, delay')
    fig.savefig(HERE / 'lab1_task2_delay.png'); plt.close(fig); print('saved', HERE / 'lab1_task2_delay.png')

    # Task 4 in simulation: path round the room, with the lidar hits.
    o, c, g, s = load(sim / 'task4-wall-following-sim')
    hx, hy = scan_hits(o, s)
    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    ax.scatter(hx, hy, s=0.6, c=AQUA, alpha=0.5, lw=0, label=LABEL['scan'], rasterized=True)
    ax.plot(o[:, 1], o[:, 2], color=BLUE, label=LABEL['path'])
    start_end(ax, o)
    finish(ax, fig, 'Lab 1 Task 4 (sim)', 'lab1_task4_sim.png', 'upper right')

    # Task 4 on the robot: only the start pose was recorded.
    o, c, g, s = load(robot / 'task4-wall-following')
    t, a0, da, r, rmin, rmax = s[len(s) // 2]
    ok = (r > rmin) & (r < rmax)
    a = a0 + da * np.arange(len(r))
    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    ax.plot(r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]), '.', color=AQUA, ms=5, label=LABEL['scan'])
    ax.add_patch(plt.Rectangle((-0.089, -0.069), 0.178, 0.138, fc='none', ec=INK, lw=1.5, label='robot'))
    ax.arrow(0, 0, 0.15, 0, head_width=0.04, color=INK)
    ax.set(xlim=(-1.5, 3.2), ylim=(-1.6, 1.6))
    finish(ax, fig, 'Lab 1 Task 4 (robot), start pose', 'lab1_task4_robot_start.png', 'lower right')

    # Task 4 against a 0.25 m block, from the desk simulation with the lab-day settings.
    if args.desk_block:
        p = np.load(args.desk_block)
        fig, ax = plt.subplots(figsize=(6.5, 6.5))
        ax.add_patch(plt.Rectangle((-0.125, -0.125), 0.25, 0.25, color=GREY, label='0.25 m block'))
        ax.plot(p[:, 0], p[:, 1], color=BLUE, lw=1.3, label=LABEL['path'] .replace('/odom', 'simulated'))
        ax.plot(p[0, 0], p[0, 1], 'o', color=INK, ms=8, label=LABEL['start'])
        finish(ax, fig, 'Lab 1 Task 4 (sim), block', 'lab1_task4_sim_block.png', 'upper left')


if __name__ == '__main__':
    main()
