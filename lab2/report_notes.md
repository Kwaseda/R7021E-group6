# Lab 2: what is wrong, and what we fixed

Written 2026-10-06 from the six bags recorded on 2026-10-05 and from offline simulation runs.
Figures are in `plots/`.

## Task 4 never ran on the robot

Both task 4 bags show the robot stationary. 0.019 m and 0.023 m of travel over 98 s and 94 s.

It was parked at (1.819, 0.504), which is exactly where `task3-final` ended. Task 4's boundary
box is +/-1.5 m, so x = 1.819 was already outside it before the first solve. The state bounds
are hard and they are enforced at every horizon step including step zero, and step zero is the
measurement, not something the optimiser can change. So the problem was infeasible from the
first tick and stayed that way.

We forgot to reset the robot between tasks. But a forgotten reset should cost a bad run, not a
controller that cannot recover. The real flaw is ours: hard bounds on a measured state.

## The same mechanism broke task 2

`task2-big-real-obstacle`: the robot drove to 0.904 m from the obstacle centre against a
1.105 m keep-out radius, so 0.2 m inside the zone, then stopped at x = 1.292 and never moved
again. Hard obstacle constraint, measurement inside it, infeasible forever.

The obstacle was a second, physically large robot that the instructor parked in our path, so a
1.0 m radius may well have been honest. Either way the geometry was unworkable: inflated for
our own footprint it becomes a 1.105 m keep-out, 2.21 m across, and detouring around that needs
roughly 2.2 m of lateral swing. Our horizon reaches 20 x 0.1 x 0.22 = 0.44 m. The solver could
not see far enough around it to commit to a detour, so it drove to the edge and stopped.

Worth stating plainly, because it is a design limit and not a tuning slip: an obstacle is only
avoidable if the detour it forces is smaller than the distance the horizon covers. At 0.22 m/s
that is 0.44 m. Nothing about the cost function changes it.

## Task 1 overshot its own boundary

Final position x = 1.061 with the box at +/-1.0. We had added a 0.05 m margin between the
commanded box and the solver's box, and 61 mm of overshoot ate straight through it. Same class
of problem again.

## Task 3 actually worked

`task3-final`: 2.229 m of path, closest approach 0.2629 m against a 0.255 m constraint. Both
obstacle constraints active and respected, 8 mm to spare. This is the run to show.

## What the instructor saw, and was right about

He suggested the problem was in the publishing, and mentioned t_step. Both land.

The publishing half is the stop-go above: gaps in `/cmd_vel` whenever a solve failed.

The t_step half is actuation delay. The command we compute at tick k does not take effect until
tick k+1 or later, but we were solving from the pose measured at tick k. The optimiser plans
from a position the robot has already left. Measured offline on the task 4 circle:

| case | mean error | jerk | omega flips | closest approach |
|---|---|---|---|---|
| no delay | 0.0023 m | 0.0044 | 4 | 0.2251 m |
| 1 step delay, uncompensated | 0.0024 m | 0.0062 | 5 | 0.2254 m |
| 1 step delay, compensated | 0.0023 m | 0.0045 | 5 | 0.2251 m |
| 2 step delay, uncompensated | 0.0033 m | **0.0192** | 11 | **0.2217 m** |
| 2 step delay, compensated | 0.0024 m | 0.0067 | 5 | 0.2254 m |

Keep-out radius is 0.225 m. At two steps of delay the uncompensated run breaks the constraint
by 3.3 mm and its jerk is 4.4 times the clean case. Compensating cuts jerk by 2.9 times and
puts the robot back outside the keep-out.

The fix is three lines: propagate the measured pose forward by `delay_steps * t_step` using the
last command before handing it to the solver. It is a `delay_steps` parameter, default 1.

## The jagged circle was not a tuning problem

We assumed the jerky motion in task 4 meant badly chosen weights. It did not. We swept
`r_input`, `lap_period`, `min_v` and `max_angular_velocity` in closed loop offline, and the
original parameters were already the best of the eight combinations tested:

| config | mean radius error | omega sign flips | stalls |
|---|---|---|---|
| lap 60 s, r_input 0.01 | 0.0023 m | 4 per lap | 0 |
| lap 35 s, r_input 0.1, min_v 0.05 | 0.0043 m | 4 per lap | 0 |

The cause was in the node, not the cost function. On any solver failure we published zero
velocity. Near a constraint the solver fails intermittently, so the robot got stop, go, stop,
go. That is the jerkiness.

Fixed by holding the last command for up to three ticks before falling back to zero. A
transient failure now coasts instead of braking, and a real failure still stops the robot
within 0.3 s.

## Task 4 fixed, in simulation

Two laps, closed loop, at a desk:

```
solver failures      0
mean radius error    0.0023 m   max 0.0458 m
omega sign flips     8 over 1200 steps
v range              0.056 to 0.107 m/s, never stalled
closest approach     0.2251 m against a 0.225 m keep-out
```

`plots/task4_simulation_twolaps.png`. That closest approach is worth reading twice: the soft
constraint spends 0.1 mm of slack. It is not buying clearance cheaply, because the penalty is
1e4 against a tracking weight of 1.0. On hardware, with actuation delay, it will spend more,
and that is the point of making it soft. A hard constraint there deadlocks; we measured that
too.

## The lidar display was accumulating

The laser looked like it was mapping everywhere the robot had been, updating slowly. That was
not the lidar. The RViz LaserScan display had `Decay Time` set to 30, so it held 30 seconds of
scans on screen at once. Set to 0, with point size raised from 0.01 to 0.03 so a single scan is
still readable. We only ever needed the current scan.

## The obstacles were invisible in RViz

Nothing published them. One of us tried to write the marker publisher during the session and
could not finish it, because it was not clear what a `Marker` actually needs to be filled in
with, so it was left commented out.

Four fields decide whether a marker appears at all, and three of them fail silently:

- `id` must be unique within the namespace, or later markers overwrite earlier ones and you
  see one obstacle instead of two.
- `pose.orientation.w = 1.0`. Left at zero the quaternion is invalid and RViz drops the marker
  with no error.
- `scale.x` and `scale.y` are the **diameter**, not the radius. Half-size circles look like a
  tolerance problem rather than a units problem.
- `color.a` above zero. Default alpha is 0, which is a fully transparent marker. The display existed in an earlier config and the topic never had a
publisher, so we were flying blind on exactly the thing that was breaking the runs. The node
now publishes a `MarkerArray` on `/mpc_obstacles` with each obstacle and its inflated keep-out
ring, and the RViz config carries the display.

If this had been there on Monday we would have seen the 1.105 m keep-out swallowing half the
room in about two seconds.

## A bug in our own analysis tool

`plot_rosbag.py` had its own copy of the task table. We ran task 2 with command-line overrides,
so the plot compared the recorded path against obstacles that were never in play, and reported
a violation against the wrong circle. It now reads the table out of the node, and takes
`--obstacle x,y,r` and `--bound` for runs that used overrides.

Two copies of the same constants is the bug. We have made that mistake before, in Lab 1, with
five YAML files.

## Task 5, driving between two obstacles

Added as task 5, since the lab only asks for at least two obstacle constraints and a gate
demonstrates them better than a slalom: both constraints are active at the same moment rather
than one after the other.

Two obstacles at (1.00, +/-0.40), radius 0.15, keep-out 0.255 m, so a 0.290 m corridor between
the keep-out edges. Goal (2.0, 0.0) from a start at the origin.

```
reached goal in 8.8 s, 0 solver failures
closest approach 0.4001 m and 0.4001 m against a 0.255 m keep-out
max |y| through the gate 0.0000 m
```

It goes straight down the middle and never touches either constraint. The symmetry works for us
here, unlike the single-obstacle case: with the gap on the start-to-goal axis, straight through
is the unique optimum, so there is no tie for the solver to fail to break.

Narrowing the gate, same geometry, goal reached every time with zero failures:

| gate offset | corridor | closest approach | constraints respected |
|---|---|---|---|
| 0.28 m | 0.050 m | 0.2802 m | yes |
| 0.30 m | 0.090 m | 0.3002 m | yes |
| 0.40 m | 0.290 m | 0.4001 m | yes |
| 0.55 m | 0.590 m | 0.5501 m | yes |

Even a 50 mm corridor works, because the model is a point and the obstacles are already
inflated by the robot's 0.105 m half-diagonal. That inflation is conservative, so 50 mm of
further clearance is real. We use 0.290 m for the demo because it is visibly tight without
looking like a stunt.

`plots/task5_gate_simulation.png`.

## What we would do differently in labs 1 to 3

**Lab 1.** We shipped two packages, five YAML parameter files and fourteen Python modules,
about 2500 lines, for three tasks. One package with two or three scripts and the parameters
declared next to the code that reads them would have been both smaller and defensible. We
could not explain our own layout when asked, which is the real measure of it being wrong.

**Lab 2.** Same lesson applied: one node, four tasks, selected by a `task` parameter against
one table of constraint sets. Changing a task is a restart, not a rebuild, because the goal is
a time-varying parameter and only the constraints are compiled in.

**Lab 3.** Fill in the template. We added packages beside the course's own and it made the
work harder to read without making it better.

## Still wrong, and we know it

- State bounds are still hard. A pose outside the box is still unrecoverable, which is exactly
  what killed task 4. The fix is four soft `set_nl_cons` instead of `mpc.bounds`, and we have
  not done it yet.
- There is no heading term in the cost, only position. On a circle the robot is free to cut the
  chord, and some of the 45 mm peak error is that rather than lag.
- The horizon is 0.44 m of travel. Any obstacle needing a bigger detour than that will be
  pushed through rather than avoided. This is a hard limit of the horizon, not a tuning choice,
  and it needs stating whenever we pick obstacle sizes.
- Task 4 has only ever completed in simulation. We have no hardware bag of it.
- `delay_steps` defaults to 1 and we have not measured the robot's actual command latency. Over
  Wi-Fi with a 5 Hz lidar it is plausibly closer to 2, which is where the uncompensated numbers
  above get bad. Measuring it is a stopwatch job: timestamp a command, timestamp the odometry
  response.
