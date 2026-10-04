# Lab 2 — the one file you have to write

**File count: one.** Everything else already exists.

```
lab2/ros2_ws/src/r7021e_lab2/r7021e_lab2/mpc_controller.py     ← you write this, ~140 lines
                             package.xml  setup.py  setup.cfg   ← done
                             resource/r7021e_lab2  __init__.py  ← done
```

No launch file. No YAML. `setup.py` already registers `mpc_node` →
`r7021e_lab2.mpc_controller:main`.

**One parameter selects the task**, so the command line stays short at 13:00:

```bash
ros2 run r7021e_lab2 mpc_node --ros-args -p task:=2
```

---

## The file, slot by slot

### Slot 1 — imports
`warnings` (to silence do-mpc's three import warnings), `do_mpc`, `casadi as ca`, `numpy as np`,
`math`, `rclpy`, `Node`, `TwistStamped`, `Pose`, `Odometry`, `Path`, `PoseStamped`,
`qos_profile_sensor_data`.

Wrap the do-mpc import:
```
with warnings.catch_warnings():
    warnings.simplefilter('ignore', UserWarning)
    import do_mpc
```

### Slot 2 — the task table (module level, above the class)

A dict, task number → constraint set. **These coordinates are measured and feasible. Do not
improvise them** — section 4 of `PLAN.md` says what breaks and why.

| task | bound | obstacles (x, y, r) | soft | goal source |
|---|---|---|---|---|
| 1 | 1.0 | none | – | `/new_position` |
| 2 | 2.0 | (0.75, 0.08, 0.30) | no | `/new_position` |
| 3 | 2.0 | (0.7, 0.15, 0.15), (1.3, −0.15, 0.15) | no | `/new_position` |
| 4 | 1.5 | (0.0, 0.62, 0.12) | **yes**, penalty 1e4 | circle |

Circle for task 4: radius 0.8, centre (0, 0), lap 60 s, 2 laps, start delay 8 s.

### Slot 3 — `build_model()`  *(notebook cell 41, plus TVP)*

Course-provided, so this goes in essentially as written:

- `do_mpc.model.Model('continuous')`
- states `_x`: `x`, `y`, `th`
- inputs `_u`: `vx`, `vt`
- **TVP** `_tvp`: `xdes`, `ydes`   ← the addition the notebook makes in cell 47
- `set_rhs('x', vx*ca.cos(th))`, `set_rhs('y', vx*ca.sin(th))`, `set_rhs('th', vt)`
- `model.setup()`, return it

`ca.cos` not `math.cos` — `th` is a symbol here.

### Slot 4 — `build_mpc(model, cfg)`  *(notebook cell 42 + cells 47–48)*

**Order matters — all of this before `setup()`:**

1. `set_param(n_horizon=N, t_step=ts, n_robust=0, store_full_solution=True,
   nlpsol_opts={'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0})`
   — `n_robust=0`, not the notebook's 1: no uncertain parameters, so 1 just enlarges the problem.
2. error term: `(model.x['x'] - model.tvp['xdes'])**2 + (model.x['y'] - model.tvp['ydes'])**2`
   → `set_objective(lterm=q_position*err, mterm=q_terminal*err)`
3. `set_rterm(vx=r_input, vt=r_input)` — penalises *changes* in input, i.e. smoothness.
4. input bounds: `vx ∈ [0.0, 0.22]`, `vt ∈ [−0.8, 0.8]`
5. state bounds: `x, y ∈ [−bound, +bound]` from the task table
6. per obstacle, radius inflated by **0.105** (the Burger's half-diagonal — the model is a point):
   `set_nl_cons('obs%d' % i, r**2 - ((x-ox)**2 + (y-oy)**2), ub=0.0, soft_constraint=soft)`
   and when soft, also `penalty_term_cons=1e4`
7. `self.tvp = mpc.get_tvp_template()` then `mpc.set_tvp_fun(lambda t_now: self.tvp)`
8. `mpc.setup()`

### Slot 5 — `__init__`

- `declare_parameter('task', 1)`, then `t_step` 0.1, `n_horizon` 20, `q_position` 1.0,
  `q_terminal` 1.0, `r_input` 0.01, `goal_tolerance` 0.05, `odom_timeout` 0.5
  (`t_step` and `n_horizon` declared separately so task 1c is a command-line change)
- look the task up in the table, build model and mpc
- publishers: `/cmd_vel` (`TwistStamped`, 10), `/mpc_prediction` (`Path`, 10)
- subscriptions: `/odom` (`Odometry`, **`qos_profile_sensor_data`**), `/new_position` (`Pose`, 10)
- timer at `t_step`
- state: `self.x = self.y = self.yaw = 0.0`, `self.goal = None`, `self.have_odom = False`,
  `self.last_odom_time = None`, `self.warm = False`, `self.t0 = None`

### Slot 6a — `on_odom(msg)`  — stores only
x, y, and yaw from the quaternion (the four-line `arctan2`). Set `have_odom = True` and record
the time for the staleness check. **No control maths here** — rule 1.

### Slot 6b — `on_goal(msg)`
`self.goal = (msg.position.x, msg.position.y)`, log `NEW GOAL`, and set `self.warm = False` so
the next tick re-warm-starts. Log it — that line is your proof the goal arrived.

### Slot 6c — `circle_point(elapsed)` — task 4 only
```
running = min(max(elapsed - start_delay, 0.0), laps*lap_period)
phase   = 2*pi*running/lap_period
return centre + radius*(cos(phase), sin(phase))
```
Clamping `running` holds the start point during the delay (so the robot drives onto the circle
first) and holds the end point after the last lap.

### Slot 7 — `control_tick()` — the only place maths happens

In this order:

1. no odom yet, or odom older than `odom_timeout` → `stop()`, return
2. **task 4**: reference point for each horizon step `k` is `circle_point(elapsed + k*t_step)`
   — the circle's *future*, which is what makes it trajectory tracking.
   **tasks 1–3**: if `self.goal is None` → return; if within `goal_tolerance` → `stop()`, return;
   otherwise the same goal in every slot.
3. write the horizon: `for k in range(n_horizon+1): self.tvp['_tvp', k, 'xdes'] = gx` (and `ydes`)
4. `state = np.array([[x], [y], [yaw]])`; if not `self.warm`: `mpc.x0 = state`,
   `mpc.set_initial_guess()`, `self.warm = True`
5. `u = mpc.make_step(state)` inside `try/except` → on exception, `stop()` and log
6. **check `mpc.solver_stats['success']`.** If false: log `return_status` and `stop()`.
   This is not optional — an MPC that fails quietly is worse than one that halts.
7. `v = u[0,0]`, `omega = u[1,0]`, clamp both to the bounds, publish `TwistStamped` with
   `header.frame_id = 'base_link'` and `header.stamp = now`
8. publish the prediction as a `Path` in frame `odom` from
   `mpc.data.prediction(('_x','x'))[0,:,0]` and the same for `'y'`

### Slot 8 — `stop()` and `main()`
`stop()` publishes an empty `TwistStamped` (zeros).
`main()`: `rclpy.init()`, node, `try: rclpy.spin(node) / except KeyboardInterrupt: pass /
finally: node.stop(); node.destroy_node(); rclpy.shutdown()`.

---

## Build and run

```bash
cd ~/R7021E-group6/lab2/ros2_ws && colcon build --symlink-install
```

```bash
source install/setup.bash
```

Gazebo in its own terminal, then the node. Goals — **`--times 6 --rate 2`, never `--once`**
(a single publish exits before the bag recorder matches it, and the goal vanishes from your
recording while the robot still moves):

```bash
ros2 topic pub --times 6 --rate 2 /new_position geometry_msgs/msg/Pose "{position: {x: 0.8, y: 0.5}}"
```

Task 1b, the goal outside the boundary — expect the robot to stop at `x = 1.0` and log the
solver status. **That is the correct answer, not a failure:** the goal is outside the feasible
set the boundary defines, so the boundary is the nearest reachable point.

Task 1c: `-p t_step:=0.2 -p n_horizon:=10`. The number that matters is neither alone — it is
`n_horizon · t_step · v_max`, the reach.

## Order of work tonight

1. Slots 1–4 and a `main()` that just builds and exits. Run it. If the solver constructs, the
   hard part is done.
2. Slots 5–8 for **task 1 only**. Get it driving in Gazebo. Record `task1`.
3. Tasks 2 and 3 — table entries, no code change. Record each.
4. Task 4 last — it needs `circle_point` and the soft constraint.
5. If task 1 is not driving two hours before you sleep, **stop and rehearse task 1 instead.**
   One task defended beats four that stall.
