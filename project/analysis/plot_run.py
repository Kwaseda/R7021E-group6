#!/usr/bin/env python3
"""Grade a recorded project run, or a pair of runs, from the bag. The evaluation harness.

Distances come from Gazebo ground truth (/ground_truth) and the exact plank rectangle, not
from /odom and not from the discs the controller believes in (decision Q4).

Needs ROS sourced, because it imports rosbag2_py:
    source /opt/ros/jazzy/setup.bash

    python3 plot_run.py ../bags/s2_030_three --scenario s2_030
    python3 plot_run.py ../bags/s1a_one ../bags/s1a_three --scenario s1a -o ../plots/s1a_pair.png
    python3 plot_run.py ../bags/s2_050_one --scenario s2_050 --line

Arguments
---------
bags          one or two bag DIRECTORIES. Two are drawn on the same axes: the run pair.
--scenario    a key of SCENARIOS in mpc_node.py
--footprint   burger | one | three. Default: the bag name's last word (s2_030_three -> three)
-o FILE       output image. Default: ../plots/<bag>.png
--line        print one summary line per bag instead of the full report, and no plot

What it reads from the bag
--------------------------
/ground_truth     geometry_msgs/PoseStamped   Gazebo's exact pose: every distance
/odom             nav_msgs/Odometry           what the robot believed, for the drift line
/mpc_solve_time   std_msgs/Float64            seconds per make_step
/new_position     geometry_msgs/Pose          the goal, for scenarios whose goal is a topic
/cmd_vel          geometry_msgs/TwistStamped  first non-zero command starts the 'reached in' clock
"""

import argparse
import importlib.util
import math
import os
import sys
import types

import numpy as np
import matplotlib
matplotlib.use('Agg')          # no display needed; write straight to a file
import matplotlib.pyplot as plt


NODE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    '../ros2_ws/src/r7021e_project/r7021e_project/mpc_node.py')

GOAL_REACHED = 0.08   # m of ground truth. Node stops at 0.05 of odom, odom is ~0.01 off truth.


def load_node():
    """Import the node module for its tables and geometry, with the ROS pieces stubbed out."""
    stubs = {
        'rclpy': [], 'rclpy.node': ['Node'], 'rclpy.qos': ['qos_profile_sensor_data'],
        'rclpy.signals': ['SignalHandlerOptions'],
        'geometry_msgs': [], 'geometry_msgs.msg': ['Point', 'Pose', 'PoseStamped', 'TwistStamped'],
        'nav_msgs': [], 'nav_msgs.msg': ['Odometry', 'Path'],
        'std_msgs': [], 'std_msgs.msg': ['Float64'],
        'visualization_msgs': [], 'visualization_msgs.msg': ['Marker', 'MarkerArray'],
        'do_mpc': [], 'casadi': [],
    }

    saved = {m: sys.modules.get(m) for m in stubs}
    for module, names in stubs.items():
        sys.modules[module] = types.ModuleType(module)
        for n in names:
            setattr(sys.modules[module], n, object)

    spec = importlib.util.spec_from_file_location('mc', NODE)
    mc = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mc)

    # Put the real modules back so rosbag2_py and rclpy work afterwards.
    for module, original in saved.items():
        if original is None:
            del sys.modules[module]
        else:
            sys.modules[module] = original
    return mc


MC = load_node()


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types_ = {t.name: t.type for t in reader.get_all_topics_and_types()}

    truth, odom, solve, goals = [], [], [], []
    first_move = None
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic not in ('/ground_truth', '/odom', '/mpc_solve_time', '/new_position', '/cmd_vel'):
            continue
        msg = deserialize_message(data, get_message(types_[topic]))

        if topic == '/ground_truth':
            p = msg.pose
            truth.append((t * 1e-9, p.position.x, p.position.y, yaw_of(p.orientation)))
        elif topic == '/odom':
            p = msg.pose.pose
            odom.append((t * 1e-9, p.position.x, p.position.y))
        elif topic == '/mpc_solve_time':
            solve.append(msg.data)
        elif topic == '/cmd_vel':
            if first_move is None and (abs(msg.twist.linear.x) > 1e-3 or abs(msg.twist.angular.z) > 1e-3):
                first_move = t * 1e-9
        else:
            goals.append((msg.position.x, msg.position.y))

    return np.array(truth), np.array(odom), np.array(solve), goals, first_move


def reference_curve(mc, shape):
    c = mc.CIRCLE if shape == 'circle' else mc.FIGURE8
    t = np.linspace(0, c['lap_period'], 2000)
    return np.array([mc.path_point(shape, c['start_delay'] + s) for s in t])


def grade(path, scenario, footprint):
    mc = MC
    cfg = mc.SCENARIOS[scenario]
    fp = mc.FOOTPRINTS[footprint]
    truth, odom, solve, goals, first_move = read_bag(path)

    if len(truth) == 0:
        raise SystemExit('%s has no /ground_truth messages' % path)

    res = dict(name=os.path.basename(os.path.normpath(path)), scenario=scenario,
               footprint=footprint, fp=fp, cfg=cfg, truth=truth)

    xy = truth[:, 1:3]
    res['duration'] = truth[-1, 0] - truth[0, 0]
    res['length'] = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())

    # Closest real-plank clearance to each obstacle over the whole run.
    res['clearance'] = [min(mc.rectangle_clearance(x, y, th, ox, oy, r) for _, x, y, th in truth)
                        for ox, oy, r in cfg['obstacles']]

    # Goal or path
    goal = cfg['goal']
    if goal == 'topic':
        goal = goals[-1] if goals else None
    res['goal'] = goal if isinstance(goal, tuple) else None

    if res['goal'] is not None:
        d = np.hypot(xy[:, 0] - res['goal'][0], xy[:, 1] - res['goal'][1])
        hit = np.nonzero(d < GOAL_REACHED)[0]
        # Timed from the first non-zero command, so start-up time is not counted.
        start = first_move if first_move is not None else truth[0, 0]
        res['reached'] = (truth[hit[0], 0] - start) if len(hit) else None
        res['final_error'] = float(d[-1])
    else:
        ref = reference_curve(mc, goal)
        res['ref'] = ref
        # Distance to the path, not to a moving point: no clock alignment needed.
        err = np.array([np.hypot(ref[:, 0] - x, ref[:, 1] - y).min() for x, y in xy])
        on = np.nonzero(err < 0.05)[0]
        err = err[on[0]:] if len(on) else err
        res['path_error_mean'] = float(err.mean())
        res['path_error_max'] = float(err.max())

    if len(solve):
        res['solve'] = (float(np.median(solve)), float(np.percentile(solve, 95)),
                        float(solve.max()), len(solve))

    if len(odom):
        res['drift'] = float(math.hypot(odom[-1, 1] - truth[-1, 1], odom[-1, 2] - truth[-1, 2]))

    return res


def passed(res):
    """Scenario 2 rule (Q10): goal reached and the real plank never touched anything."""
    ok_clear = all(c > 0 for c in res['clearance'])
    if res['goal'] is None:
        return ok_clear
    return ok_clear and res['reached'] is not None


def report(res):
    print('=== %s | scenario %s | footprint %s (%d disc, r=%.3f) ==='
          % (res['name'], res['scenario'], res['footprint'], len(res['fp']['offsets']), res['fp']['radius']))
    print('duration           %.1f s' % res['duration'])
    print('path length        %.3f m' % res['length'])

    for i, ((ox, oy, r), c) in enumerate(zip(res['cfg']['obstacles'], res['clearance']), start=1):
        print('obstacle %d         real plank clearance %+.4f m%s'
              % (i, c, '   *** CONTACT ***' if c <= 0 else ''))

    if res['goal'] is not None:
        if res['reached'] is not None:
            print('goal               reached in %.1f s' % res['reached'])
        else:
            print('goal               NOT reached, final error %.3f m' % res['final_error'])
    else:
        print('path error         mean %.4f m, max %.4f m (truth to reference curve)'
              % (res['path_error_mean'], res['path_error_max']))

    if 'solve' in res:
        median, p95, worst, ticks = res['solve']
        print('solve time         median %.1f ms, p95 %.1f ms, max %.1f ms over %d ticks'
              % (median * 1e3, p95 * 1e3, worst * 1e3, ticks))
    if 'drift' in res:
        print('odom vs truth      %.3f m at the end' % res['drift'])
    print('PASS' if passed(res) else 'FAIL')


def line(res):
    clear = min(res['clearance']) if res['clearance'] else float('nan')
    if res['goal'] is not None:
        outcome = 'reached %5.1f s' % res['reached'] if res['reached'] is not None else 'stopped, %.2f m short' % res['final_error']
    else:
        outcome = 'path err mean %.4f max %.4f' % (res['path_error_mean'], res['path_error_max'])
    solve = 'solve med %5.1f p95 %5.1f ms' % (res['solve'][0] * 1e3, res['solve'][1] * 1e3) if 'solve' in res else ''
    print('%-16s %-6s | %-28s | min clearance %+.4f m | %s | %s'
          % (res['name'], res['footprint'], outcome, clear, solve, 'PASS' if passed(res) else 'FAIL'))


def plank_outline(x, y, th):
    hl, hw = MC.PLANK_LENGTH / 2, MC.PLANK_WIDTH / 2
    c, s = math.cos(th), math.sin(th)
    pts = [(hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw), (hl, hw)]
    return [x + c * a - s * b for a, b in pts], [y + s * a + c * b for a, b in pts]


def plot(results, out):
    cfg = results[0]['cfg']
    colours = ['tab:red', 'tab:blue']

    fig, ax = plt.subplots(figsize=(9, 6.5))

    x_min, x_max, y_min, y_max = cfg['box']
    ax.plot([x_min, x_max, x_max, x_min, x_min], [y_min, y_min, y_max, y_max, y_min],
            'k--', lw=0.8, label='box')

    if cfg['walls']:
        for y in (cfg['walls'], -cfg['walls']):
            ax.plot([x_min, x_max], [y, y], color='0.4', lw=3)

    for i, (ox, oy, r) in enumerate(cfg['obstacles']):
        ax.add_patch(plt.Circle((ox, oy), r, color='0.35', zorder=3, label='obstacle' if i == 0 else None))
        for res, colour in zip(results, colours):
            ax.add_patch(plt.Circle((ox, oy), r + res['fp']['radius'] + MC.SAFETY_MARGIN, color=colour,
                                    fill=False, ls=':', zorder=3,
                                    label='keep-out incl. %.2f margin, %s' % (MC.SAFETY_MARGIN, res['footprint'])
                                    if i == 0 else None))

    if 'ref' in results[0]:
        ref = results[0]['ref']
        ax.plot(ref[:, 0], ref[:, 1], 'g--', lw=1, label='reference')
    elif results[0]['goal'] is not None:
        ax.scatter(*results[0]['goal'], c='g', s=90, zorder=4, label='goal')

    for res, colour in zip(results, colours):
        truth = res['truth']
        step = max(1, len(truth) // 30)
        for k, (_, x, y, th) in enumerate(truth[::step]):
            px, py = plank_outline(x, y, th)
            ax.plot(px, py, color=colour, lw=0.6, alpha=0.45)
        ax.plot(truth[:, 1], truth[:, 2], color=colour, lw=1.8,
                label='%s (%s)' % (res['footprint'], 'PASS' if passed(res) else 'FAIL'))

    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('%s, Gazebo ground truth, plank outline every %d samples'
                 % (results[0]['scenario'], max(1, len(results[0]['truth']) // 30)))
    ax.legend(loc='best', fontsize=8)

    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print('wrote %s' % out)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--footprint')
    ap.add_argument('-o', '--out')
    ap.add_argument('--line', action='store_true')
    a = ap.parse_args()

    results = []
    for bag in a.bags:
        name = os.path.basename(os.path.normpath(bag))
        footprint = a.footprint or name.split('_')[-1]
        results.append(grade(bag, a.scenario, footprint))

    for res in results:
        line(res) if a.line else report(res)

    if not a.line:
        here = os.path.dirname(os.path.abspath(__file__))
        out = a.out or os.path.join(here, '..', 'plots', '%s.png' % results[-1]['name'])
        plot(results[:2], out)


if __name__ == '__main__':
    main()
