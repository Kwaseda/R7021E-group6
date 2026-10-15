#!/usr/bin/env python3
"""R7021E project: nonlinear MPC for a TurtleBot3 Burger carrying a plank, built with do-mpc.

Built on the Lab 2 controller (lab2/.../mpc_controller.py). What is new:

  - the body is a chain of discs along the plank, chosen by the footprint parameter
  - the boundary box is four soft constraints instead of hard state bounds
  - one SCENARIOS table, read by this node, the launch file, the desk sim and the analysis

    ros2 run r7021e_project mpc_node --ros-args -p scenario:=s2_030 -p footprint:=three -p use_sim_time:=true

Build-only smoke test (source ROS first):

    python3 mpc_node.py --smoke
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
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Point, Pose, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray


# ---------------------------------------------------------------- the body

PLANK_LENGTH = 0.50     # m, along the heading (decision Q2)
PLANK_WIDTH = 0.20      # m
BURGER_RADIUS = 0.105   # m, Lab 2's single disc for the bare robot

# Radius of the circle the plank sweeps when the robot turns on the spot: half its diagonal.
PLANK_SWEEP = math.hypot(PLANK_LENGTH / 2, PLANK_WIDTH / 2)   # 0.269 m


def disc_chain(n):
    """n equal discs covering the plank, centred on the axle (decision Q3).

    Cut the plank into n slices along its length. Each slice is (L/n) x W and its covering
    disc has half the slice's diagonal as radius. The centres sit on the body x-axis.
    """
    radius = math.hypot(PLANK_LENGTH / (2 * n), PLANK_WIDTH / 2)

    slice_length = PLANK_LENGTH / n
    offsets = tuple(-PLANK_LENGTH / 2 + slice_length * (i + 0.5) for i in range(n))

    return offsets, radius


# offsets: disc centres along the body x-axis [m].  radius: every disc's radius [m].
# sweep: what the soft box is shrunk by, the real body's turn-on-the-spot radius.
FOOTPRINTS = {
    'burger': dict(offsets=(0.0,), radius=BURGER_RADIUS, sweep=BURGER_RADIUS),
    'one': dict(offsets=disc_chain(1)[0], radius=disc_chain(1)[1], sweep=PLANK_SWEEP),
    'three': dict(offsets=disc_chain(3)[0], radius=disc_chain(3)[1], sweep=PLANK_SWEEP),
}


def rectangle_clearance(x, y, th, ox, oy, r_obs):
    """Distance from the real plank rectangle to an obstacle circle's edge. <= 0 is contact.

    Not used by the controller, which only knows discs. The desk sim and the analysis use it
    to grade a run against the true shape (decision Q4).
    """
    # Obstacle centre in the plank's own frame: undo the translation, then the rotation.
    dx = ox - x
    dy = oy - y
    px = math.cos(th) * dx + math.sin(th) * dy
    py = -math.sin(th) * dx + math.cos(th) * dy

    # Nearest point of the rectangle to that centre.
    qx = max(-PLANK_LENGTH / 2, min(PLANK_LENGTH / 2, px))
    qy = max(-PLANK_WIDTH / 2, min(PLANK_WIDTH / 2, py))

    return math.hypot(px - qx, py - qy) - r_obs


# ---------------------------------------------------------------- the scenarios

POST_RADIUS = 0.10                                 # m, scenario 2 posts
GAPS = (0.60, 0.50, 0.40, 0.34, 0.30, 0.28)        # m, between post surfaces (decision Q10)

# box: (x_min, x_max, y_min, y_max) the body must stay inside, soft.
# obstacles: (x, y, radius) circles.  walls: |y| of the scenario 2 corridor walls, or None.
# goal: 'topic' (/new_position), 'circle', 'figure8', or a fixed (x, y).
SCENARIOS = {
    # Lab 2 tasks, kept so the copy can be checked against Lab 2 (gate 1).
    'lab2_t3': dict(box=(-2.0, 2.0, -2.0, 2.0), walls=None, goal='topic',
                    obstacles=[(0.70, 0.15, 0.15), (1.30, -0.15, 0.15)]),
    'lab2_t4': dict(box=(-2.5, 2.5, -2.5, 2.5), walls=None, goal='circle',
                    obstacles=[(0.00, 0.62, 0.12)]),

    # Scenario 1: known goal path, an obstacle the footprint has to go round (decision Q9).
    's1a': dict(box=(-2.0, 2.0, -2.0, 2.0), walls=None, goal='circle',
                obstacles=[(0.00, 0.62, 0.12)]),
    's1b': dict(box=(-2.0, 2.0, -2.0, 2.0), walls=None, goal='figure8',
                obstacles=[(0.00, 0.36, 0.08)]),   # start outside both keep-outs (LOG.md)
}

# Scenario 2: a corridor with two posts touching the walls, the gap between them on the axis.
for _gap in GAPS:
    _post_y = _gap / 2 + POST_RADIUS
    _wall_y = _gap / 2 + 2 * POST_RADIUS

    SCENARIOS['s2_%03d' % round(_gap * 100)] = dict(
        box=(-0.5, 2.5, -_wall_y, _wall_y), walls=_wall_y, goal=(2.0, 0.0), gap=_gap,
        obstacles=[(1.0, _post_y, POST_RADIUS), (1.0, -_post_y, POST_RADIUS)])

CIRCLE = dict(radius=0.8, centre_x=0.0, centre_y=0.0,
              lap_period=60.0, laps=1, start_delay=8.0)

FIGURE8 = dict(size=0.8, lap_period=45.0, laps=1, start_delay=2.0)


def path_point(shape, elapsed):
    """Reference on a known path. Holds the start point during start_delay, walks the path,
    then holds the final point."""
    c = CIRCLE if shape == 'circle' else FIGURE8

    span = c['laps'] * c['lap_period']
    running = min(max(elapsed - c['start_delay'], 0.0), span)
    phase = 2.0 * math.pi * running / c['lap_period']

    if shape == 'circle':
        return (c['centre_x'] + c['radius'] * math.cos(phase),
                c['centre_y'] + c['radius'] * math.sin(phase))

    # Lemniscate of Gerono: crosses itself at the origin.
    return (c['size'] * math.sin(phase),
            c['size'] * math.sin(phase) * math.cos(phase))


# ---------------------------------------------------------------- the MPC

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


def build_TBmpc(model, t_step, n_horizon, min_v, max_v, max_w, box, sweep,
                obstacles, offsets, disc_radius, penalty, q_position, q_terminal, r_input):
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
        nlpsol_opts={'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0},
    )

    x = model.x['x']
    y = model.x['y']
    th = model.x['th']

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

    # Every soft constraint below costs penalty * slack; slack is zero whenever it can be.
    soft = dict(ub=0.0, soft_constraint=True, penalty_term_cons=penalty)

    # Boundary box, soft (decision Q5). Hard bounds on x, y made a measurement outside the
    # box unrecoverable in Lab 2. Shrunk by the body's sweep so it fits at any heading.
    x_min, x_max, y_min, y_max = box
    mpc.set_nl_cons('box_x_max', x - (x_max - sweep), **soft)
    mpc.set_nl_cons('box_x_min', (x_min + sweep) - x, **soft)
    mpc.set_nl_cons('box_y_max', y - (y_max - sweep), **soft)
    mpc.set_nl_cons('box_y_min', (y_min + sweep) - y, **soft)

    # Keep-out: one constraint per obstacle per disc. Each disc centre turns with the heading.
    # Squared on both sides, so there is no square root.  r^2 - d^2 <= 0  means outside.
    for j, (ox, oy, r_obs) in enumerate(obstacles, start=1):
        keep_out = r_obs + disc_radius

        for i, d in enumerate(offsets, start=1):
            cx = x + d * ca.cos(th)
            cy = y + d * ca.sin(th)

            mpc.set_nl_cons('obs%d_disc%d' % (j, i),
                            keep_out ** 2 - ((cx - ox) ** 2 + (cy - oy) ** 2),
                            **soft)

    tvp_template = mpc.get_tvp_template()
    mpc.set_tvp_fun(lambda t_now: tvp_template)

    mpc.setup()
    return mpc, tvp_template


def smoke_test():
    """Build the solver for a few scenarios and solve once. If this prints OK, setup is ok."""
    for name, footprint in (('lab2_t3', 'burger'), ('s1a', 'one'), ('s2_030', 'three')):
        cfg = SCENARIOS[name]
        fp = FOOTPRINTS[footprint]

        started = time.perf_counter()
        model = build_TBmodel()
        mpc, tvp = build_TBmpc(model, 0.1, 40, 0.0, 0.22, 0.8, cfg['box'], fp['sweep'],
                               cfg['obstacles'], fp['offsets'], fp['radius'],
                               1e4, 1.0, 1.0, 0.01)

        for k in range(41):
            tvp['_tvp', k, 'xdes'] = 1.0
            tvp['_tvp', k, 'ydes'] = 0.0

        state = np.array([[0.0], [0.0], [0.0]])
        mpc.x0 = state
        mpc.set_initial_guess()
        u = mpc.make_step(state)

        print('%-8s %-6s built+solved in %5.2f s | success=%s | v=%6.3f omega=%6.3f'
              % (name, footprint, time.perf_counter() - started,
                 mpc.solver_stats['success'], float(u[0, 0]), float(u[1, 0])))


# ---------------------------------------------------------------- the node

class MpcNode(Node):

    def __init__(self):
        super().__init__('mpc_node')

        # What to run
        self.declare_parameter('scenario', 's1a')        # a key of SCENARIOS
        self.declare_parameter('footprint', 'three')     # burger | one | three

        # Solver
        self.declare_parameter('t_step', 0.1)            # s, control period = prediction step
        self.declare_parameter('n_horizon', 20)          # steps; reach 20 x 0.1 x 0.22 = 0.44 m, Lab 2 value (desk sweep, LOG.md)
        self.declare_parameter('q_position', 1.0)        # stage cost on position error
        self.declare_parameter('q_terminal', 1.0)        # terminal cost on position error
        self.declare_parameter('r_input', 0.01)          # cost on input changes: smoothness
        self.declare_parameter('obstacle_penalty', 10000.0)   # price per unit of soft slack

        # Robot limits
        self.declare_parameter('max_linear_velocity', 0.22)   # m/s, the Burger's real top speed
        self.declare_parameter('max_angular_velocity', 0.8)   # rad/s, Lab 2 value
        self.declare_parameter('min_linear_velocity', 0.0)    # m/s, no reversing

        # Behaviour
        self.declare_parameter('goal_tolerance', 0.05)   # m, stop inside this of a fixed goal
        self.declare_parameter('odom_timeout', 0.5)      # s, stop if odometry is older
        self.declare_parameter('hold_ticks', 3)          # failed solves to coast through
        self.declare_parameter('delay_steps', 1)         # ticks of actuation delay to predict over

        p = lambda name: self.get_parameter(name).value

        self.scenario = str(p('scenario'))
        self.footprint = str(p('footprint'))
        self.t_step = float(p('t_step'))
        self.n_horizon = int(p('n_horizon'))
        self.max_v = float(p('max_linear_velocity'))
        self.max_w = float(p('max_angular_velocity'))
        self.min_v = float(p('min_linear_velocity'))
        self.goal_tolerance = float(p('goal_tolerance'))
        self.odom_timeout = float(p('odom_timeout'))
        self.hold_ticks = int(p('hold_ticks'))
        self.delay_steps = int(p('delay_steps'))

        if self.scenario not in SCENARIOS:
            raise ValueError('scenario must be one of %s' % sorted(SCENARIOS))
        if self.footprint not in FOOTPRINTS:
            raise ValueError('footprint must be one of %s' % sorted(FOOTPRINTS))

        cfg = SCENARIOS[self.scenario]
        fp = FOOTPRINTS[self.footprint]

        self.box = cfg['box']
        self.obstacles = cfg['obstacles']
        self.goal_source = cfg['goal']
        self.offsets = fp['offsets']
        self.disc_radius = fp['radius']

        # The solver, built once
        self.model = build_TBmodel()
        self.mpc, self.tvp = build_TBmpc(
            self.model, self.t_step, self.n_horizon,
            self.min_v, self.max_v, self.max_w, self.box, fp['sweep'],
            self.obstacles, self.offsets, self.disc_radius,
            float(p('obstacle_penalty')),
            float(p('q_position')), float(p('q_terminal')), float(p('r_input')))

        # Publishers
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.prediction_pub = self.create_publisher(Path, '/mpc_prediction', 10)
        self.obstacle_pub = self.create_publisher(MarkerArray, '/mpc_obstacles', 10)
        self.footprint_pub = self.create_publisher(MarkerArray, '/mpc_footprint', 10)
        self.solve_time_pub = self.create_publisher(Float64, '/mpc_solve_time', 10)

        # Subscriptions
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Pose, '/new_position', self.on_goal, 10)

        # Timers
        self.create_timer(1.0, self.publish_obstacles)
        self.create_timer(self.t_step, self.control_tick)

        # State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.goal = cfg['goal'] if isinstance(cfg['goal'], tuple) else None
        self.last_odom = None
        self.warm = False
        self.t0 = None
        self.checked = False
        self.held = 0
        self.last_cmd = (0.0, 0.0)
        self.reach = self.n_horizon * self.t_step * self.max_v

        self.get_logger().info(
            'scenario %s | footprint %s: %d disc(s) of r=%.3f | horizon %d x %.2f s | reach %.2f m'
            % (self.scenario, self.footprint, len(self.offsets), self.disc_radius,
               self.n_horizon, self.t_step, self.reach))
        for i, (ox, oy, radius) in enumerate(self.obstacles, start=1):
            self.get_logger().info(
                'obstacle %d at (%.3f, %.3f) r=%.3f, keep-out %.3f m'
                % (i, ox, oy, radius, radius + self.disc_radius))

    # ------------------------------------------------------------ callbacks: store only

    def on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        self.last_odom = self.get_clock().now()

    def on_goal(self, msg):
        if self.goal_source != 'topic':
            self.get_logger().warn('scenario %s has its own goal, /new_position ignored'
                                   % self.scenario)
            return

        x_min, x_max, y_min, y_max = self.box
        gx = max(x_min, min(x_max, msg.position.x))
        gy = max(y_min, min(y_max, msg.position.y))

        self.goal = self.push_out(gx, gy)
        self.warm = False
        self.get_logger().info('NEW GOAL x=%.3f y=%.3f' % self.goal)

    # ------------------------------------------------------------ helpers

    def push_out(self, gx, gy):
        """Move a goal that sits inside an inflated obstacle to the nearest point outside it."""
        for ox, oy, radius in self.obstacles:
            keep_out = radius + self.disc_radius + 0.02
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
            keep_out = radius + self.disc_radius
            d = math.hypot(self.x - ox, self.y - oy)

            if d < keep_out:
                self.get_logger().warn(
                    'obstacle %d: robot is %.3f m from its centre, inside the %.3f m keep-out. '
                    'Soft constraint will drive it out.' % (i, d, keep_out))

    def horizon_points(self):
        """(x, y) reference for each of the N+1 horizon steps, or None to hold still."""
        if self.goal_source in ('circle', 'figure8'):
            if self.t0 is None:
                self.t0 = self.get_clock().now()

            elapsed = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
            return [path_point(self.goal_source, elapsed + k * self.t_step)
                    for k in range(self.n_horizon + 1)]

        if self.goal is None:
            return None
        if math.hypot(self.goal[0] - self.x, self.goal[1] - self.y) <= self.goal_tolerance:
            return None

        return [self.goal] * (self.n_horizon + 1)

    # ------------------------------------------------------------ the algorithm

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

        self.publish_footprint(now)

        points = self.horizon_points()
        if points is None:
            self.stop()
            return

        for k, (gx, gy) in enumerate(points):
            self.tvp['_tvp', k, 'xdes'] = gx
            self.tvp['_tvp', k, 'ydes'] = gy

        # The command decided now is executed delay_steps ticks from now, so solve from
        # where the robot will be by then, not where it was measured.
        px, py, pyaw = self.x, self.y, self.yaw
        v0, w0 = self.last_cmd
        for _ in range(self.delay_steps):
            px += self.t_step * v0 * math.cos(pyaw)
            py += self.t_step * v0 * math.sin(pyaw)
            pyaw += self.t_step * w0

        state = np.array([[px], [py], [pyaw]])
        if not self.warm:
            self.mpc.x0 = state
            self.mpc.set_initial_guess()
            self.warm = True

        started = time.perf_counter()
        try:
            u = self.mpc.make_step(state)
        except Exception as error:
            self.get_logger().error('solver raised: %s' % error)
            self.stop()
            return
        self.solve_time_pub.publish(Float64(data=time.perf_counter() - started))

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

    # ------------------------------------------------------------ publishing

    def publish_cmd(self, now, v, omega):
        cmd = TwistStamped()
        cmd.header.stamp = now.to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

    def marker(self, ns, mid, kind, stamp):
        """A Marker with the four fields that fail silently already filled in."""
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid                    # unique within ns, or markers overwrite each other
        m.type = kind
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0    # zero quaternion is invalid and RViz drops it
        m.color.a = 1.0               # alpha 0 is invisible
        return m

    def publish_obstacles(self):
        """Each obstacle and its keep-out ring for this footprint, and the box."""
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, (ox, oy, radius) in enumerate(self.obstacles):
            for j, (r, alpha) in enumerate(((radius, 0.8), (radius + self.disc_radius, 0.25))):
                m = self.marker('obstacles', i * 2 + j, Marker.CYLINDER, stamp)
                m.pose.position.x = float(ox)
                m.pose.position.y = float(oy)
                m.pose.position.z = 0.05
                m.scale.x = m.scale.y = float(2.0 * r)   # diameter, not radius
                m.scale.z = 0.1
                m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.3, 0.1, alpha
                arr.markers.append(m)

        x_min, x_max, y_min, y_max = self.box
        m = self.marker('box', 0, Marker.LINE_STRIP, stamp)
        m.scale.x = 0.02
        m.color.r, m.color.g, m.color.b = 0.8, 0.1, 0.1
        for bx, by in ((x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max), (x_min, y_min)):
            m.points.append(Point(x=float(bx), y=float(by), z=0.0))
        arr.markers.append(m)

        self.obstacle_pub.publish(arr)

    def publish_footprint(self, now):
        """The plank and the discs the controller believes in, at the measured pose."""
        arr = MarkerArray()
        stamp = now.to_msg()
        half_yaw = self.yaw / 2.0

        plank = self.marker('footprint', 0, Marker.CUBE, stamp)
        plank.pose.position.x = self.x
        plank.pose.position.y = self.y
        plank.pose.position.z = 0.21
        plank.pose.orientation.z = math.sin(half_yaw)
        plank.pose.orientation.w = math.cos(half_yaw)
        plank.scale.x = PLANK_LENGTH
        plank.scale.y = PLANK_WIDTH
        plank.scale.z = 0.01
        plank.color.r, plank.color.g, plank.color.b, plank.color.a = 0.85, 0.55, 0.2, 0.9
        arr.markers.append(plank)

        for i, d in enumerate(self.offsets, start=1):
            m = self.marker('footprint', i, Marker.CYLINDER, stamp)
            m.pose.position.x = self.x + d * math.cos(self.yaw)
            m.pose.position.y = self.y + d * math.sin(self.yaw)
            m.pose.position.z = 0.22
            m.scale.x = m.scale.y = 2.0 * self.disc_radius
            m.scale.z = 0.005
            m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.3, 0.9, 0.35
            arr.markers.append(m)

        self.footprint_pub.publish(arr)

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


# ---------------------------------------------------------------- main

def main():
    # Without this, Ctrl-C shuts rclpy down before the finally block runs, stop() cannot
    # publish, and the robot keeps its last command. Lab 2's node had that bug.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
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
