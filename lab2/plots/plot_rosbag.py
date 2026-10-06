#!/usr/bin/env python3
"""Plot a Lab 2 run from a recorded rosbag.

Adapted from animate_trajectory() in the course notebook (do_mpc.ipynb cell 34).
The notebook version animates a list of [x, y] it already had in memory; this one
reads the same information back out of a bag, and knows about the boundary box and
the task's obstacles.

Needs ROS sourced, because it imports rosbag2_py:
    source /opt/ros/jazzy/setup.bash

    python3 plot_rosbag.py ../bags/task2 --task 2
    python3 plot_rosbag.py ../bags/task4 --task 4 --animate

Arguments
---------
bag          path to the bag DIRECTORY (the folder holding metadata.yaml)
--task N     1-4. Selects the obstacles and boundary drawn, from TASKS below.
-o FILE      output image. Default: <bagname>_task<N>.png
--animate    also write an animated GIF beside the PNG
--interval   GIF frame interval in ms (default 60)

Outputs
-------
A PNG (always) and optionally a GIF. Prints the numbers you need for the report:
path length, duration, final position, closest approach to each obstacle, and
mean tracking error against the goal.

What it reads from the bag
--------------------------
/odom            nav_msgs/Odometry      the path actually driven
/new_position    geometry_msgs/Pose     the goals you published (tasks 1-3)
/mpc_prediction  nav_msgs/Path          the last predicted horizon, if recorded
"""

import argparse
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')          # no display needed; write straight to a file
import matplotlib.pyplot as plt
import matplotlib.animation as animation


# Read straight out of the node, so the two can never drift apart.
import importlib.util as _u
_spec = _u.spec_from_file_location('_node', os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '../ros2_ws/src/r7021e_lab2/r7021e_lab2/mpc_controller.py'))
_src = open(_spec.origin).read()
_ns = {'__name__': '_node'}
exec(compile(_src[:_src.index('class MpcNode')].replace('import rclpy', 'pass')
             .replace('from rclpy', '#from rclpy').replace('from geometry_msgs', '#from geometry_msgs')
             .replace('from nav_msgs', '#from nav_msgs').replace('from visualization_msgs', '#from visualization_msgs')
             .replace('from rcl_interfaces', '#from rcl_interfaces'), _spec.origin, 'exec'), _ns)
ROBOT_INFLATION = _ns['ROBOT_INFLATION']
TASKS = {k: dict(bound=v['bound'], obstacles=v['obstacles']) for k, v in _ns['TASKS'].items()}
CIRCLE = dict(radius=_ns['CIRCLE']['radius'], centre_x=_ns['CIRCLE']['centre_x'],
              centre_y=_ns['CIRCLE']['centre_y'])


def read_bag(path):
    """Return {topic: [(t_seconds, message), ...]} for the topics we care about."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        sys.exit('error: ROS is not sourced.  Run:  source /opt/ros/jazzy/setup.bash')

    if not os.path.isdir(path):
        sys.exit('error: %s is not a directory. Pass the bag FOLDER, not the .mcap file.' % path)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id=''),
        rosbag2_py.ConverterOptions('', ''),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = ('/odom', '/new_position', '/mpc_prediction')
    out = {t: [] for t in wanted}
    t0 = None

    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic not in out:
            continue
        if t0 is None:
            t0 = stamp
        msg = deserialize_message(raw, get_message(types[topic]))
        out[topic].append(((stamp - t0) * 1e-9, msg))
    return out


def extract(bag):
    """Pull the arrays out of the raw messages."""
    odom = bag['/odom']
    t = np.array([s for s, _ in odom])
    xy = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for _, m in odom])

    goals, goal_t = [], []
    for s, m in bag['/new_position']:
        p = (m.position.x, m.position.y)
        if not goals or p != goals[-1]:      # only the distinct ones
            goals.append(p)
            goal_t.append(s)

    pred = []
    if bag['/mpc_prediction']:
        last = bag['/mpc_prediction'][-1][1]
        pred = [(p.pose.position.x, p.pose.position.y) for p in last.poses]

    return t, xy, goals, goal_t, pred


def report(t, xy, goals, obstacles, task):
    """Print the numbers that go in the report."""
    if len(xy) < 2:
        print('warning: fewer than two /odom samples - is the bag empty?')
        return
    steps = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    print('samples            %d over %.1f s' % (len(xy), t[-1] - t[0]))
    print('path length        %.3f m' % steps.sum())
    print('final position     x=%.3f  y=%.3f' % (xy[-1, 0], xy[-1, 1]))
    print('extent             x [%.3f, %.3f]  y [%.3f, %.3f]'
          % (xy[:, 0].min(), xy[:, 0].max(), xy[:, 1].min(), xy[:, 1].max()))

    for i, (ox, oy, r) in enumerate(obstacles, start=1):
        d = np.linalg.norm(xy - np.array([ox, oy]), axis=1)
        inflated = r + ROBOT_INFLATION
        if d.min() < r:
            note = '   *** HIT THE OBSTACLE ***'
        elif d.min() < inflated:
            note = '   soft slack used %.1f mm of the %.0f mm inflation' % (
                (inflated - d.min()) * 1000.0, ROBOT_INFLATION * 1000.0)
        else:
            note = '   clear by %.1f mm' % ((d.min() - inflated) * 1000.0)
        print('obstacle %d         closest approach %.4f m against a %.3f m constraint radius%s'
              % (i, d.min(), inflated, note))

    if task == 4:
        rad = np.linalg.norm(xy - np.array([CIRCLE['centre_x'], CIRCLE['centre_y']]), axis=1)
        err = np.abs(rad - CIRCLE['radius'])
        print('circle tracking    mean error %.4f m, max %.4f m' % (err.mean(), err.max()))
    elif goals:
        gx, gy = goals[-1]
        print('final goal error   %.4f m' % math.hypot(xy[-1, 0] - gx, xy[-1, 1] - gy))


def draw_static(ax, bound, obstacles, goals, pred, task):
    """Everything that does not move: boundary, obstacles, goals, reference."""
    ax.plot([-bound, bound, bound, -bound, -bound],
            [-bound, -bound, bound, bound, -bound],
            'r--', lw=1, label='boundary')

    for i, (ox, oy, r) in enumerate(obstacles):
        ax.add_patch(plt.Circle((ox, oy), r, color='0.35', zorder=3,
                                label='obstacle' if i == 0 else None))
        ax.add_patch(plt.Circle((ox, oy), r + ROBOT_INFLATION, color='0.35',
                                fill=False, ls=':', zorder=3,
                                label='inflated by %.3f m' % ROBOT_INFLATION if i == 0 else None))

    if task == 4:
        a = np.linspace(0, 2 * math.pi, 240)
        ax.plot(CIRCLE['centre_x'] + CIRCLE['radius'] * np.cos(a),
                CIRCLE['centre_y'] + CIRCLE['radius'] * np.sin(a),
                'g--', lw=1, label='reference circle')
    elif goals:
        g = np.array(goals)
        ax.scatter(g[:, 0], g[:, 1], c='g', marker='o', s=90, zorder=4, label='goal')

    if pred:
        p = np.array(pred)
        ax.plot(p[:, 0], p[:, 1], color='orange', lw=2, alpha=0.9,
                label='last predicted horizon')

    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--task', type=int, required=True, choices=sorted(TASKS))
    ap.add_argument('-o', '--out')
    ap.add_argument('--animate', action='store_true')
    ap.add_argument('--obstacle', action='append', default=[],
                    help='override as x,y,r  (repeatable, for runs that used -p overrides)')
    ap.add_argument('--bound', type=float)
    ap.add_argument('--interval', type=int, default=60)
    args = ap.parse_args()

    cfg = dict(TASKS[args.task])
    if args.obstacle:
        cfg['obstacles'] = [tuple(float(v) for v in o.split(',')) for o in args.obstacle]
    if args.bound:
        cfg['bound'] = args.bound
    bag = read_bag(args.bag.rstrip('/'))
    t, xy, goals, goal_t, pred = extract(bag)
    if len(xy) == 0:
        sys.exit('error: no /odom messages in the bag. Was it recorded on the right domain?')

    name = os.path.basename(args.bag.rstrip('/'))
    out = args.out or '%s_task%d.png' % (name, args.task)

    print('=== %s, task %d ===' % (name, args.task))
    report(t, xy, goals, cfg['obstacles'], args.task)

    lim = cfg['bound'] + 0.3
    fig, ax = plt.subplots(figsize=(7, 7))
    draw_static(ax, cfg['bound'], cfg['obstacles'], goals, pred, args.task)
    ax.plot(xy[:, 0], xy[:, 1], 'b-', lw=1.6, label='driven path')
    ax.scatter(xy[0, 0], xy[0, 1], c='c', marker='o', s=90, zorder=5, label='start')
    ax.scatter(xy[-1, 0], xy[-1, 1], c='b', marker='X', s=110, zorder=5, label='end')
    ax.set_title('Lab 2 task %d - %s' % (args.task, name))
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.legend(loc='best', fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=160)
    print('wrote %s' % out)

    if args.animate:
        gif = os.path.splitext(out)[0] + '.gif'
        fig2, ax2 = plt.subplots(figsize=(7, 7))
        draw_static(ax2, cfg['bound'], cfg['obstacles'], goals, [], args.task)
        ax2.set_xlim(-lim, lim)
        ax2.set_ylim(-lim, lim)
        ax2.set_title('Lab 2 task %d - %s' % (args.task, name))
        ax2.legend(loc='best', fontsize=8)
        line, = ax2.plot([], [], 'b-', lw=1.6)
        dot, = ax2.plot([], [], 'bo', ms=8)

        stride = max(1, len(xy) // 300)        # cap at ~300 frames
        frames = range(0, len(xy), stride)

        def update(i):
            line.set_data(xy[:i + 1, 0], xy[:i + 1, 1])
            dot.set_data([xy[i, 0]], [xy[i, 1]])
            return line, dot

        anim = animation.FuncAnimation(fig2, update, frames=frames,
                                       interval=args.interval, blit=True)
        anim.save(gif, writer=animation.PillowWriter(fps=max(1, 1000 // args.interval)))
        print('wrote %s' % gif)


if __name__ == '__main__':
    main()
