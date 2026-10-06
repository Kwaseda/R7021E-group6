#!/usr/bin/env python3
"""Lab 2: nonlinear MPC for the TurtleBot3 Burger, built with do-mpc.

One node serves all four tasks. The task number selects a constraint set:

    ros2 run r7021e_lab2 mpc_node --ros-args -p task:=2

Build-only smoke test (source ROS first):

    python3 mpc_controller.py --smoke
"""

import math
import sys
import time
import warnings

# do-mpc warns about optional extras (ONNX, OPC UA, approximate MPC) it does not ship.
with warnings.catch_warnings():
    warnings.simplefilter('ignore', UserWarning)
    import do_mpc

import casadi as ca
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray

# ----------------------------------------------------------------

ROBOT_INFLATION = 0.105   # m, the Burger's half-diagonal
BOUND_MARGIN = 0.05       # m, solver box is this much larger than the commanded box

TASKS = {
    1: dict(bound=1.0, obstacles=[],                                          soft=True, goal='topic'),
    2: dict(bound=4.0, obstacles=[(2.0, 0.50, 1.0)],                        soft=True, goal='topic'),
    3: dict(bound=2.0, obstacles=[(0.70, 0.15, 0.15), (1.30, -0.15, 0.15)],   soft=True, goal='topic'),
    4: dict(bound=2.5, obstacles=[(0.00, 0.62, 0.12)],                        soft=True,  goal='circle'),
}

CIRCLE = dict(radius=0.8, centre_x=0.0, centre_y=0.0,
              lap_period=60.0, laps=2, start_delay=8.0)

# ----------------------------------------------------------------
def build_TBmodel():
    """The unicycle model, with the setpoint as a time-varying parameter.

    States (_x):  x, y, th        Inputs (_u):  vx, vt
    TVP (_tvp):   xdes, ydes

    The setpoint is a _tvp rather than a constant so the goal can change
    without rebuilding the solver, which takes seconds.
    """
    model = do_mpc.model.Model('continuous')

    model.set_variable(var_type='_x', var_name='x')
    model.set_variable(var_type='_x', var_name='y')
    th = model.set_variable(var_type='_x', var_name='th')

    vx = model.set_variable(var_type='_u', var_name='vx')
    vt = model.set_variable(var_type='_u', var_name='vt')

    model.set_variable(var_type='_tvp', var_name='xdes')
    model.set_variable(var_type='_tvp', var_name='ydes')

    # ca.cos, not math.cos: th is a CasADi symbol here, not a number.
    model.set_rhs('x', vx * ca.cos(th))
    model.set_rhs('y', vx * ca.sin(th))
    model.set_rhs('th', vt)

    model.setup()
    return model

# ----------------------------------------------------------------
def build_TBmpc(model, t_step, n_horizon, min_v, max_v, max_w, bound,
              obstacles, soft, penalty, q_position, q_terminal, r_input):
    """The controller. Everything here must happen before mpc.setup().

    Returns (mpc, tvp_template). Write the goal into tvp_template each tick;
    do-mpc reads it on every make_step.
    """
    mpc = do_mpc.controller.MPC(model)

    mpc.set_param(
        n_horizon=n_horizon,
        t_step=t_step,
        n_robust=0,
        store_full_solution=True,
        nlpsol_opts={'ipopt.print_level': 1, 'ipopt.sb': 'yes', 'print_time': 0},
    )

    x = model.x['x']
    y = model.x['y']

    error = (x - model.tvp['xdes']) ** 2 + (y - model.tvp['ydes']) ** 2

    # lterm is summed over the horizon, mterm applies to the last step only.
    mpc.set_objective(lterm=q_position * error, mterm=q_terminal * error)
    # set_rterm penalises changes in the inputs, not their size: smoothness.
    mpc.set_rterm(vx=r_input, vt=r_input)

    # Input constraints: actuator limits.
    mpc.bounds['lower', '_u', 'vx'] = min_v
    mpc.bounds['upper', '_u', 'vx'] = max_v
    mpc.bounds['lower', '_u', 'vt'] = -max_w
    mpc.bounds['upper', '_u', 'vt'] = max_w

    # Boundary constraints: the box the robot must stay inside.
    mpc.bounds['lower', '_x', 'x'] = -bound
    mpc.bounds['upper', '_x', 'x'] = bound
    mpc.bounds['lower', '_x', 'y'] = -bound
    mpc.bounds['upper', '_x', 'y'] = bound

    # Keep-out circles. Squared on both sides, so there is no square root.
    # if d >= r,  i.e. stay outside.
    for i, (ox, oy, radius) in enumerate(obstacles, start=1):
        inflated = radius + ROBOT_INFLATION
        options = {'ub': 0.0, 'soft_constraint': soft}
        if soft:
            options['penalty_term_cons'] = penalty
        mpc.set_nl_cons('obs%d' % i,
                        inflated ** 2 - ((x - ox) ** 2 + (y - oy) ** 2),
                        **options)

    tvp_template = mpc.get_tvp_template()
    mpc.set_tvp_fun(lambda t_now: tvp_template)

    mpc.setup()
    return mpc, tvp_template

def circle_point(elapsed):
    """Task 4 reference. Holds the start point during start_delay so the robot
    can drive onto the circle, then walks it, then holds the final point."""
    c = CIRCLE
    span = c['laps'] * c['lap_period']
    running = min(max(elapsed - c['start_delay'], 0.0), span)
    phase = 2.0 * math.pi * running / c['lap_period']
    return (c['centre_x'] + c['radius'] * math.cos(phase),
            c['centre_y'] + c['radius'] * math.sin(phase))

def smoke_test():
    """Build the solver and exit. If this prints OK, setup is ok."""
    for task in sorted(TASKS):
        cfg = TASKS[task]
        started = time.perf_counter()
        model = build_TBmodel()
        mpc, tvp = build_TBmpc(model, 0.1, 20, 0.0, 0.22, 0.8, cfg['bound'],
                             cfg['obstacles'], cfg['soft'], 1e4, 1.0, 1.0, 0.01)
        for k in range(21):
            tvp['_tvp', k, 'xdes'] = 1.0
            tvp['_tvp', k, 'ydes'] = 0.0
        state = np.array([[0.0], [0.0], [0.0]])
        mpc.x0 = state
        mpc.set_initial_guess()
        u = mpc.make_step(state)
        ok = mpc.solver_stats['success']
        print('task %d: built in %5.2f s | first solve success=%s | v=%6.3f omega=%6.3f | %s'
              % (task, time.perf_counter() - started, ok,
                 float(u[0, 0]), float(u[1, 0]), mpc.solver_stats['return_status']))



# ----------------------------------------------------------------
class MpcNode(Node):

    def __init__(self):
        super().__init__('mpc_node')

        self.declare_parameter('task', 1)
        self.declare_parameter('t_step', 0.1)
        self.declare_parameter('n_horizon', 100)
        self.declare_parameter('max_linear_velocity', 0.22)
        self.declare_parameter('max_angular_velocity', 0.8)
        self.declare_parameter('min_linear_velocity', 0.0)
        self.declare_parameter('q_position', 1.0)
        self.declare_parameter('q_terminal', 1.0)
        self.declare_parameter('r_input', 0.01)
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('odom_timeout', 0.5)
        self.declare_parameter('obstacle_penalty', 10000.0)
        self.declare_parameter('hold_ticks', 3)

        p = lambda name: self.get_parameter(name).value

        self.task = int(p('task'))
        self.t_step = float(p('t_step'))
        self.n_horizon = int(p('n_horizon'))
        self.max_v = float(p('max_linear_velocity'))
        self.max_w = float(p('max_angular_velocity'))
        self.min_v = float(p('min_linear_velocity'))
        self.goal_tolerance = float(p('goal_tolerance'))
        self.odom_timeout = float(p('odom_timeout'))

        if self.task not in TASKS:
            raise ValueError('task must be one of %s' % sorted(TASKS))
        darr = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        self.declare_parameter('bound', 0.0)
        self.declare_parameter('obstacle_x', [], darr)
        self.declare_parameter('obstacle_y', [], darr)
        self.declare_parameter('obstacle_radius', [], darr)

        cfg = TASKS[self.task]
        self.goal_source = cfg['goal']

        self.bound = float(p('bound')) or cfg['bound']

        ox = list(p('obstacle_x'))
        oy = list(p('obstacle_y'))
        orad = list(p('obstacle_radius'))
        if ox or oy or orad:
            if not len(ox) == len(oy) == len(orad):
                raise ValueError('obstacle_x, obstacle_y and obstacle_radius must be equal length')
            obstacles = list(zip(ox, oy, orad))
        else:
            obstacles = cfg['obstacles']
        self.obstacles = obstacles

        self.model = build_TBmodel()
        self.mpc, self.tvp = build_TBmpc(
            self.model, self.t_step, self.n_horizon,
            self.min_v, self.max_v, self.max_w, self.bound + BOUND_MARGIN,
            obstacles, cfg['soft'], float(p('obstacle_penalty')),
            float(p('q_position')), float(p('q_terminal')), float(p('r_input')))

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.prediction_pub = self.create_publisher(Path, '/mpc_prediction', 10)
        self.obstacle_pub = self.create_publisher(MarkerArray, '/mpc_obstacles', 10)
        self.create_timer(1.0, self.publish_obstacles)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Pose, '/new_position', self.on_goal, 10)
        self.create_timer(self.t_step, self.control_tick)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.goal = None
        self.last_odom = None
        self.warm = False
        self.t0 = None
        self.checked = False
        self.hold_ticks = int(p('hold_ticks'))
        self.held = 0
        self.last_cmd = (0.0, 0.0)
        self.reach = self.n_horizon * self.t_step * self.max_v

        self.get_logger().info(
            'task %d | bound +/-%.2f | %s constraint | goal from %s | '
            'reach = %d x %.2f x %.2f = %.2f m'
            % (self.task, self.bound, 'soft' if cfg['soft'] else 'hard',
               self.goal_source, self.n_horizon, self.t_step, self.max_v, self.reach))
        for i, (ox, oy, radius) in enumerate(self.obstacles, start=1):
            self.get_logger().info(
                'obstacle %d at (%.2f, %.2f) r=%.3f, keep-out %.3f m'
                % (i, ox, oy, radius, radius + ROBOT_INFLATION))



    def on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        self.last_odom = self.get_clock().now()


    def on_goal(self, msg):

    
        raw = (msg.position.x, msg.position.y)
        clip = lambda v: max(-self.bound, min(self.bound, v))
        self.goal = self.push_out(clip(raw[0]), clip(raw[1]))
        self.warm = False
        if self.goal != raw:
            self.get_logger().info(
                'NEW GOAL x=%.3f y=%.3f requested, outside the +/-%.2f box, '
                'clipped to x=%.3f y=%.3f' % (raw[0], raw[1], self.bound, *self.goal))
        else:
            self.get_logger().info('NEW GOAL x=%.3f y=%.3f' % self.goal)


    def push_out(self, gx, gy):
        """Move a goal that sits inside an inflated obstacle to the nearest point outside it."""
        for ox, oy, radius in self.obstacles:
            keep_out = radius + ROBOT_INFLATION + 0.02
            d = math.hypot(gx - ox, gy - oy)
            if d < keep_out:
                if d < 1e-6:
                    gx, gy = ox + keep_out, oy
                else:
                    gx = ox + (gx - ox) / d * keep_out
                    gy = oy + (gy - oy) / d * keep_out
                self.get_logger().warn(
                    'goal was inside an obstacle, moved to x=%.3f y=%.3f' % (gx, gy))
        return (gx, gy)

    def check_start(self):
        """Log anything that will make the first solve hard, once, at startup."""
        for i, (ox, oy, radius) in enumerate(self.obstacles, start=1):
            keep_out = radius + ROBOT_INFLATION
            d = math.hypot(self.x - ox, self.y - oy)
            if d < keep_out:
                self.get_logger().warn(
                    'obstacle %d: robot is %.3f m from its centre, inside the %.3f m keep-out. '
                    'Soft constraint will drive it out; move the robot if it struggles.'
                    % (i, d, keep_out))
            if keep_out > self.bound:
                self.get_logger().warn(
                    'obstacle %d keep-out radius %.3f m is larger than the %.2f m box. '
                    'Raise bound.' % (i, keep_out, self.bound))
            if 2.0 * keep_out > self.reach:
                self.get_logger().warn(
                    'obstacle %d needs more detour than %.2f m of horizon reach. '
                    'Raise n_horizon or t_step.' % (i, self.reach))

    def horizon_points(self):
        """(x, y) reference for each of the N+1 horizon steps, or None to hold still."""
        if self.goal_source == 'circle':
            if self.t0 is None:
                self.t0 = self.get_clock().now()
            elapsed = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
            return [circle_point(elapsed + k * self.t_step) for k in range(self.n_horizon + 1)]

        if self.goal is None:
            return None
        if math.hypot(self.goal[0] - self.x, self.goal[1] - self.y) <= self.goal_tolerance:
            return None
        return [self.goal] * (self.n_horizon + 1)

    def control_tick(self):
        now = self.get_clock().now()
        if self.last_odom is None:
            return
        if (now - self.last_odom).nanoseconds * 1e-9 > self.odom_timeout:
            self.get_logger().warn('odometry is stale, stopping', throttle_duration_sec=2.0)
            self.stop()
            return

        if not self.checked:
            self.checked = True
            self.check_start()

        points = self.horizon_points()
        if points is None:
            self.stop()
            return

        for k, (gx, gy) in enumerate(points):
            self.tvp['_tvp', k, 'xdes'] = gx
            self.tvp['_tvp', k, 'ydes'] = gy

        state = np.array([[self.x], [self.y], [self.yaw]])
        if not self.warm:
            self.mpc.x0 = state
            self.mpc.set_initial_guess()
            self.warm = True

        try:
            u = self.mpc.make_step(state)
        except Exception as error:
            self.get_logger().error('solver raised: %s' % error)
            self.stop()
            return

        if not self.mpc.solver_stats['success']:
            self.get_logger().warn('solver failed: %s' % self.mpc.solver_stats['return_status'],
                                   throttle_duration_sec=1.0)
            self.warm = False
            if self.held < self.hold_ticks:
                self.held += 1
                self.publish_cmd(now, *self.last_cmd)
            else:
                self.stop()
            return

        v = max(self.min_v, min(self.max_v, float(u[0, 0])))
        omega = max(-self.max_w, min(self.max_w, float(u[1, 0])))
        self.held = 0
        self.last_cmd = (v, omega)
        self.publish_cmd(now, v, omega)
        self.publish_prediction(now)

        """   
    ef publish_obstacles(self):
        # Drawn at the inflated radius, which is what the solver actually enforces.
        markers = MarkerArray()
        for i, (ox, oy, radius) in enumerate(self.obstacles):
            m = Marker()
            m.header.frame_id = 'odom'
            m.ns = 'mpc_obstacles'
            m.id = i
            m.type = Marker.CYLINDER
            m.pose.position.x = ox
            m.pose.position.y = oy
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 2.0 * radius
            m.scale.z = 0.2
            m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.2, 0.2, 0.35
            markers.markers.append(m)
        if markers.markers:
            self.obstacle_pub.publish(markers)

        """

    def publish_cmd(self, now, v, omega):
        cmd = TwistStamped()
        cmd.header.stamp = now.to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

    def publish_obstacles(self):
        """Draw each obstacle and its inflated keep-out ring in RViz."""
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, (ox, oy, radius) in enumerate(self.obstacles):
            for j, (r, alpha) in enumerate(((radius, 0.8), (radius + ROBOT_INFLATION, 0.25))):
                m = Marker()
                m.header.frame_id = 'odom'
                m.header.stamp = stamp
                m.ns = 'obstacles'
                m.id = i * 2 + j
                m.type = Marker.CYLINDER
                m.action = Marker.ADD
                m.pose.position.x = float(ox)
                m.pose.position.y = float(oy)
                m.pose.position.z = 0.05
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = float(2.0 * r)
                m.scale.z = 0.1
                m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.3, 0.1, alpha
                arr.markers.append(m)
        self.obstacle_pub.publish(arr)

    def publish_prediction(self, now):
        try:
            xs = self.mpc.data.prediction(('_x', 'x'))[0, :, 0]
            ys = self.mpc.data.prediction(('_x', 'y'))[0, :, 0]
        except Exception:
            return
        path = Path()
        path.header.stamp = now.to_msg()
        path.header.frame_id = 'odom'
        for px, py in zip(xs, ys):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(px)
            pose.pose.position.y = float(py)
            path.poses.append(pose)
        self.prediction_pub.publish(path)

    def stop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        self.cmd_pub.publish(cmd)

# ----------------------------------------------------------------

def main():
    rclpy.init()
    node = MpcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    if '--smoke' in sys.argv:
        smoke_test()
    else:
        main()
