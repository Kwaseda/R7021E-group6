#!/usr/bin/env python3
"""Run a task in closed loop at a desk and plot it like plot_rosbag.py does.

The simulation counterpart of plot_rosbag.py, so a simulated run and a recorded
run can be compared side by side with the same axes and the same overlays. No
ROS, no Gazebo: it integrates the unicycle model the controller predicts with.

    python3 sim_task.py --task 3 --goal 1.8,0.0
    python3 sim_task.py --task 4 --laps 2
    python3 sim_task.py --task 3 --goal 1.8,0.0 --start 0,0,0 -o mine.png

Arguments
---------
--task N     1-4, reads bounds and obstacles from the node's own TASKS table
--goal x,y   goal for tasks 1-3. Ignored for task 4, which follows its circle.
--start x,y,theta   initial pose, default 0,0,0
--seconds S  how long to simulate, default 60 (task 4 uses --laps instead)
--laps N     task 4 only, default 2
-o FILE      output image, default task<N>_simulation.png
"""

import argparse, math, os, sys, types, importlib.util, warnings
warnings.simplefilter('ignore')
import numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt

NODE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    '../ros2_ws/src/r7021e_lab2/r7021e_lab2/mpc_controller.py')

def load_node():
    """Import the node module with the ROS pieces stubbed out."""
    for n in ('rclpy', 'rclpy.node', 'rclpy.qos', 'geometry_msgs', 'geometry_msgs.msg',
              'nav_msgs', 'nav_msgs.msg', 'rcl_interfaces', 'rcl_interfaces.msg',
              'visualization_msgs', 'visualization_msgs.msg'):
        sys.modules.setdefault(n, types.ModuleType(n))
    sys.modules['rclpy.node'].Node = object
    sys.modules['rclpy.qos'].qos_profile_sensor_data = None
    for n in ('Pose', 'PoseStamped', 'TwistStamped'):
        setattr(sys.modules['geometry_msgs.msg'], n, object)
    for n in ('Odometry', 'Path'):
        setattr(sys.modules['nav_msgs.msg'], n, object)
    for n in ('ParameterDescriptor', 'ParameterType'):
        setattr(sys.modules['rcl_interfaces.msg'], n, object)
    for n in ('Marker', 'MarkerArray'):
        setattr(sys.modules['visualization_msgs.msg'], n, object)
    spec = importlib.util.spec_from_file_location('mc', NODE)
    mc = importlib.util.module_from_spec(spec)
    sys.modules['mc'] = mc
    spec.loader.exec_module(mc)
    return mc


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--task', type=int, required=True)
    ap.add_argument('--goal', default='1.8,0.0')
    ap.add_argument('--start', default='0,0,0')
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--laps', type=int, default=2)
    ap.add_argument('--t_step', type=float, default=0.1)
    ap.add_argument('--n_horizon', type=int, default=20)
    ap.add_argument('-o', '--out')
    a = ap.parse_args()

    mc = load_node()
    cfg = mc.TASKS[a.task]
    TS, N = a.t_step, a.n_horizon
    V_MAX, W_MAX, V_MIN = 0.22, 0.8, 0.0
    circle = cfg['goal'] == 'circle'
    goal = tuple(float(v) for v in a.goal.split(','))
    p = np.array([float(v) for v in a.start.split(',')])

    if circle:
        mc.CIRCLE.update(laps=a.laps, start_delay=0.0)
        steps = int(a.laps * mc.CIRCLE['lap_period'] / TS)
    else:
        steps = int(a.seconds / TS)

    model = mc.build_TBmodel()
    mpc, tvp = mc.build_TBmpc(model, TS, N, V_MIN, V_MAX, W_MAX,
                              cfg['bound'] + mc.BOUND_MARGIN, cfg['obstacles'],
                              cfg['soft'], 1e4, 1.0, 1.0, 0.01)
    mpc.x0 = p.reshape(3, 1)
    mpc.set_initial_guess()

    traj, last, fails, reached = [p[:2].copy()], (0.0, 0.0), 0, None
    for i in range(steps):
        el = i * TS
        for k in range(N + 1):
            gx, gy = mc.circle_point(el + k * TS) if circle else goal
            tvp['_tvp', k, 'xdes'] = gx
            tvp['_tvp', k, 'ydes'] = gy
        # same one-step delay compensation the node applies
        meas = p + TS * np.array([last[0] * math.cos(p[2]), last[0] * math.sin(p[2]), last[1]])
        u = mpc.make_step(meas.reshape(3, 1))
        if not mpc.solver_stats['success']:
            fails += 1
        v = float(np.clip(u[0, 0], V_MIN, V_MAX))
        w = float(np.clip(u[1, 0], -W_MAX, W_MAX))
        last = (v, w)
        p = p + TS * np.array([v * math.cos(p[2]), v * math.sin(p[2]), w])
        traj.append(p[:2].copy())
        if not circle and reached is None and math.hypot(p[0] - goal[0], p[1] - goal[1]) < 0.05:
            reached = i * TS
            break

    traj = np.array(traj)
    steps_taken = np.linalg.norm(np.diff(traj, axis=0), axis=1)
    print('=== task %d, simulation ===' % a.task)
    print('samples            %d over %.1f s' % (len(traj), len(traj) * TS))
    print('path length        %.3f m' % steps_taken.sum())
    print('final position     x=%.3f  y=%.3f' % (traj[-1, 0], traj[-1, 1]))
    print('extent             x [%.3f, %.3f]  y [%.3f, %.3f]'
          % (traj[:, 0].min(), traj[:, 0].max(), traj[:, 1].min(), traj[:, 1].max()))
    print('solver failures    %d' % fails)
    for i, (ox, oy, r) in enumerate(cfg['obstacles'], start=1):
        d = np.linalg.norm(traj - np.array([ox, oy]), axis=1)
        keep = r + mc.ROBOT_INFLATION
        if d.min() < r:
            note = '   *** HIT THE OBSTACLE ***'
        elif d.min() < keep:
            note = '   soft slack used %.1f mm of the %.0f mm inflation' % (
                (keep - d.min()) * 1000.0, mc.ROBOT_INFLATION * 1000.0)
        else:
            note = '   clear by %.1f mm' % ((d.min() - keep) * 1000.0)
        print('obstacle %d         closest approach %.4f m against a %.3f m constraint radius%s'
              % (i, d.min(), keep, note))
    if circle:
        rad = np.linalg.norm(traj, axis=1)
        err = np.abs(rad - mc.CIRCLE['radius'])
        print('circle tracking    mean error %.4f m, max %.4f m' % (err.mean(), err.max()))
    elif reached:
        print('reached goal in    %.1f s' % reached)
    else:
        print('reached goal       no, final error %.3f m'
              % math.hypot(traj[-1, 0] - goal[0], traj[-1, 1] - goal[1]))

    b = cfg['bound']
    lim = b + 0.3
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot([-b, b, b, -b, -b], [-b, -b, b, b, -b], 'r--', lw=1, label='boundary')
    for i, (ox, oy, r) in enumerate(cfg['obstacles']):
        ax.add_patch(plt.Circle((ox, oy), r, color='0.35', zorder=3,
                                label='obstacle' if i == 0 else None))
        ax.add_patch(plt.Circle((ox, oy), r + mc.ROBOT_INFLATION, color='0.35', fill=False,
                                ls=':', zorder=3,
                                label='inflated by %.3f m' % mc.ROBOT_INFLATION if i == 0 else None))
    if circle:
        t = np.linspace(0, 2 * math.pi, 300)
        ax.plot(mc.CIRCLE['radius'] * np.cos(t), mc.CIRCLE['radius'] * np.sin(t),
                'g--', lw=1, label='reference circle')
    else:
        ax.scatter([goal[0]], [goal[1]], c='g', marker='o', s=90, zorder=4, label='goal')
    ax.plot(traj[:, 0], traj[:, 1], 'b-', lw=1.6, label='simulated path')
    ax.scatter(traj[0, 0], traj[0, 1], c='c', marker='o', s=90, zorder=5, label='start')
    ax.scatter(traj[-1, 0], traj[-1, 1], c='b', marker='X', s=110, zorder=5, label='end')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_title('Lab 2 task %d - simulation' % a.task)
    ax.legend(loc='best', fontsize=8)
    fig.tight_layout()
    out = a.out or 'task%d_simulation.png' % a.task
    fig.savefig(out, dpi=160)
    print('wrote %s' % out)


if __name__ == '__main__':
    main()
