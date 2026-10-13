# Lab 3 learning notes

Running notes from the code walkthrough. Add each new topic after discussing it, then
re-read the section from memory before moving on.

## Code-reading framework: PITSTOP

For each function or block, ask:

1. **Purpose:** What job does it do?
2. **Inputs:** What data or messages does it use?
3. **Trigger:** What calls it?
4. **State:** What state does it read or change?
5. **Transformation:** What work does it do to the inputs?
6. **Output:** What value or message does it produce?
7. **Path onward:** Who uses that output next?

## 60-second system picture

**Sensor → map → frontier → plan → drive.** Each arrow is a ROS topic or TF transform:

1. **Robot/Gazebo** publishes `/scan` and TF. Wheel odometry supplies the moving
   `odom → base_link` transform.
2. **`slam_toolbox`** matches scans using odometry/TF. It publishes the growing `/map` and
   the correcting `map → odom` transform. Together, those transforms give the robot's
   corrected pose in the map.
3. **`frontier_detector_node`** reads `/map` and publishes `/frontiers`: free cells beside
   unknown space.
4. **`navigation_node`** reads `/map`, `/frontiers` and the robot's map-frame TF pose. For
   automatic exploration it grows RRT* through known, inflated-safe free space, scores tree
   leaves for information gain, and publishes the chosen `/path`. For Task 2, an RViz
   **2D Goal Pose** click sends `/goal_pose` and the planner instead finds the cheapest
   tree branch near that known goal.
5. **`path_follower_node`** reads `/path`, TF and `/scan`; it publishes stamped velocity
   commands on `/cmd_vel`. Those commands move the robot, which supplies more scans, and
   the loop repeats.

**One line to remember:** SLAM says *where the robot is and what it has seen*; the frontier
detector says *where known space meets unknown*; navigation chooses *where to go and a
route*; the follower turns that route into *safe motion*.

RViz shows the map and planning evidence; it is not another planner. The launch file starts
these nodes together. With `task1:=true`, `task1_baseline` replaces `navigation_node` but
still feeds `/path` to the same follower. Use `use_sim_time:=true` in Gazebo only.

### Fast reminder: what to say for each task

- **Task 1:** send a `nav_msgs/Path`; the follower handles velocity.
- **Task 2:** select RViz **2D Goal Pose**, click known free map, confirm `goal from RViz`,
  then show the tree/path and arrival.
- **Task 3:** inflate map walls before planning so the point-like tree leaves room for the
  robot; `/inflated_map` marks known blocked/inflated cells as 100, not safe cells. The
  follower also has a laser stop/slow-down.
- **Task 4:** score tree leaves with `H = L - w*I + c*turn`; `I` is nearby frontier cells.
- **Task 5:** repeat plan → follow → arrival/stall/replan until the stop condition.

### Which notes to trust today

- [`CODE.md`](./CODE.md) is the compact current-source explanation; verify numbers in
  [`navigation_node.py`](../ros2_ws/src/r7021e_exploration/r7021e_exploration/navigation_node.py).
- [`RUNSHEET.md`](../RUNSHEET.md) is the current command sequence. Confirm the actual robot
  number/domain before entering it.
- [`LOG.md`](../LOG.md) separates observed runs and limitations. Do not turn an incomplete
  run into a success claim.
- [`CODEMAP.md`](./CODEMAP.md), [`THEORY.md`](./THEORY.md), [`QUESTIONS.md`](./QUESTIONS.md),
  [`COMMANDS.md`](./COMMANDS.md) and [`ORAL.md`](./ORAL.md) contain historical planner
  material. Their warnings are important: some old function names, sample log fields,
  parameter values and run instructions do not match the current NBV source. Use their
  general concepts only unless checked against current code.

## Task 1: `task1_baseline.py`

Lab instructions: [`../Lab_Instructions_Exploration.pdf`](../Lab_Instructions_Exploration.pdf)

Source: [`../ros2_ws/src/r7021e_exploration/r7021e_exploration/task1_baseline.py`](../ros2_ws/src/r7021e_exploration/r7021e_exploration/task1_baseline.py)

### How it starts

- `package.xml` identifies the package as `ament_python` and lists ROS message dependencies.
- `setup.py` registers the executable name `task1_baseline` to call
  `r7021e_exploration.task1_baseline:main`.
- Building/installing the package creates the ROS executable wrapper. The launch file uses
  that executable name; it does not run `setup.py` or build the package.
- `exploration.launch.py` starts `task1_baseline` when `task1:=true`, instead of
  `navigation_node`. It still starts `path_follower_node`, SLAM and RViz.
- `sim/maze_world.launch.py` is separate: it starts Gazebo and the robot. The package launch
  starts the ROS exploration stack.
- `use_sim_time` defaults to `false`. Keep it false on the real robot. Set it true in Gazebo
  so the nodes use Gazebo's `/clock`.

### The runtime chain

`main()` starts and spins the node. A 0.5 s timer calls `_tick()`. `_tick()` asks TF for the
robot pose, and when it has one, calls `_send()` to publish a `nav_msgs/Path` on `/path`.
`path_follower_node` receives the path and publishes `TwistStamped` commands on `/cmd_vel`.
The baseline sends paths; it does not command the robot's velocity directly.

### File walkthrough

- **Imports and class:** `math` provides distance, trigonometry and rounding helpers.
  `PoseStamped` and `Path` describe the route. `rclpy` provides the ROS node, timer and
  event loop. `tf2_ros` provides transform lookup.
- **`__init__`:** names the ROS node `task1_baseline`; declares `length` (0.5 m) and
  `tolerance` (0.10 m) parameters; fixes waypoint `spacing` at 0.10 m; creates the Path
  publisher and TF buffer/listener; initializes `start`, `end`, `target` and `legs`; and
  registers `_tick` as a repeating 0.5 s timer callback.
- **`_pose()`:** asks TF for the latest transform from `base_link` into `map`, waiting up to
  0.5 s. It extracts map-frame x and y, converts the quaternion orientation to planar yaw,
  and returns one tuple `(x, y, yaw)`. If lookup raises an exception, it returns `None`.
  The broad `except Exception` does not log the cause, so a persistent lookup failure can
  look like a node quietly waiting.
- **`_send(a, b)`:** treats `a` and `b` as the two endpoint tuples. It calculates
  `n = max(1, ceil(distance(a, b) / spacing))`, creates a `Path` in frame `map`, and adds
  `n + 1` evenly spaced poses, including both endpoints. It gives the poses the direction
  from `a` to `b`, publishes the Path, stores `b` as the current target, increments the leg
  counter and logs the endpoints.
- **`_tick()`:** gets the pose. If TF is unavailable, it returns and the timer tries again
  later. On the first usable pose it stores `(x, y)` as `start`, computes `end` 0.5 m ahead
  along the current yaw, and sends the first leg. On later ticks it compares the current
  position with `target`; once within 0.10 m it sends a leg to the opposite fixed endpoint.
- **`main()`:** initializes ROS, constructs the node and calls `rclpy.spin(node)` so timers
  and callbacks continue to run. On Ctrl-C it exits the spin loop, destroys the node and
  shuts down ROS. This file does not itself publish a stop command in its `finally` block;
  the path follower has its own stop-on-shutdown behavior in the normal launch.

### Timer distinction

`create_timer(0.5, self._tick)` registers `_tick` as a repeating callback. `rclpy.spin(node)`
processes ROS events and invokes `_tick` whenever the timer is ready. `_tick` does not return
a path to the timer. Instead, `_tick` may call `_send`, and `_send` publishes the path.

### Numeric example

If the robot starts at `(1.0, 2.0)` facing +x, then `end = (1.5, 2.0)`. With a 0.5 m leg and
0.10 m spacing, `n = 5`, so the Path has six poses:

| Index `i` | Position `(x, y)` |
|---:|---|
| 0 | `(1.0, 2.0)` |
| 1 | `(1.1, 2.0)` |
| 2 | `(1.2, 2.0)` |
| 3 | `(1.3, 2.0)` |
| 4 | `(1.4, 2.0)` |
| 5 | `(1.5, 2.0)` |

There are five intervals, which require six points including both endpoints. If the robot
faces +y instead, x stays 1.0 and y runs from 2.0 to 2.5 in 0.1 m steps.

On the return leg, `_tick()` calls `_send(end, start)`. Here `a` is the new leg's start
(the old `end`) and `b` is its destination (the original `start`). `_send()` generates the
intermediate poses in reverse order. `target` stores only the destination endpoint, not the
list of poses.

### Task 1 behavior and next step

The PDF asks for a new Path after the robot reaches the end of the previous one. This
implementation repeatedly alternates between the fixed start and end points. It does so
when the measured robot position comes within the tolerance of the current target.

## Navigation planner: first-pass map

Current source: [`../ros2_ws/src/r7021e_exploration/r7021e_exploration/navigation_node.py`](../ros2_ws/src/r7021e_exploration/r7021e_exploration/navigation_node.py)

### `Grid`: map coordinates and array indices

An `OccupancyGrid` stores cells in a flat row-major list. Reshaping to NumPy shape
`(height, width)` means indexing is `data[row, column]`: the first index selects a y-row,
and the second selects an x-column. `Grid.cell(x, y)` therefore returns
`(row, column)`, calculating row from y and column from x. This is not an opposite ordering:
it is the array's row/column indexing corresponding to the map's y/x axes.

`Grid.world(row, column)` reverses the conversion and returns the centre of that cell in
world coordinates. It adds half a cell because integer indices identify cells, while the
planner works with continuous x/y positions.

For example, with origin `(-5, -5)`, resolution `0.05 m`, and width `200`, world point
`(0, 0)` gives `row = floor((0 - (-5)) / 0.05) = 100` and
`column = floor((0 - (-5)) / 0.05) = 100`. Its flat row-major index is
`row * width + column = 100 * 200 + 100 = 20100`.

### `free_mask`: known free space with inflated obstacles

`free_mask(grid, radius)` returns a NumPy Boolean array with the same height and width as the
map. `True` means a possible location for the robot's centre; `False` means the planner must
not sample or travel through that cell.

1. `grid.data > 50` identifies occupied cells.
2. `k = ceil(radius / resolution)` converts the physical padding radius to whole cells. With
   `0.105 m` and `0.05 m/cell`, `k = 3`.
3. The nested offset loops OR the occupied mask into a grown mask for offsets inside a disc
   of radius `k` cells. `np.pad` supplies a false border while shifting cells at map edges.
4. The return expression keeps only values from 0 through 50 that are not in the grown
   obstacle mask. Unknown values (`-1`) are excluded.

This is configuration-space inflation: the tree treats the robot as a point, so growing
walls by approximately the robot radius gives that point a clearance margin. Unknown is not
assumed safe. The current function does not despeckle/remove isolated occupied cells; claims
about a despeckle stage in older theory notes do not match this source.

### `segment_free`: check a tree edge for collision

`segment_free(mask, grid, a, b)` has a different purpose from Task 1's `_send(a, b)`.
`_send()` samples a route into waypoints for the follower; it does not check whether the
route is safe. `segment_free()` checks whether the straight connection proposed by RRT*
crosses only allowed cells. A free endpoint `b` is not sufficient: the line from `a` to `b`
could pass through an inflated wall cell between its endpoints.

It converts sampled x/y positions along the line to grid row/column indices and returns
`False` if any index is outside the map or its mask cell is false. The code chooses
`n = max(1, int(distance / (resolution / 2)))` intervals. This aims for half-cell sampling,
but `int` rounds down, so the actual spacing can be slightly larger than half a cell when
the distance is not an exact multiple. Do not describe it as a strict maximum spacing.

### `rrt_star`: grow a tree and improve its branches

Diagram: [`rrt-star-walkthrough.svg`](./rrt-star-walkthrough.svg)

The tree is held in three parallel lists: `nodes[i]` is a point, `parent[i]` is the index
of its parent (`-1` for the root), and `cost[i]` is the stored cost from the root. It starts
with the root, no parent and zero cost.

Each iteration samples a cell from `free_mask`, converts it to that cell's world-space
centre, finds the nearest current tree node, and steers toward the sample by at most
`step_size` (0.30 m). It asks `segment_free()` to reject a blocked edge. Around a valid new
point, it finds existing nodes within `rewire_radius` (0.60 m), chooses the collision-free
parent that gives the lowest stored path cost, then adds the new node.

RRT* then checks whether connecting each nearby node through the new one would lower its
cost. If so, it changes that node's parent and stored cost. In this source, it does not
propagate the changed cost to that node's existing descendants. Therefore some descendant
costs can be stale after rewiring; the tree's parent links may describe a path whose stored
cost is not the true current path length. This is a limitation to know when explaining how
the planner selects the lowest-cost branch.

There is no goal-biased sample in this implementation. The callback `enough(nodes, parent)`
is checked every 500 iterations once the initial 1500-iteration budget has been reached;
the hard maximum is 8000. For Task 2, `enough` tests whether a node is within 0.15 m of the
clicked goal. For NBV, it tests whether a leaf sees at least `min_gain` frontier cells.

### How `max_iterations` and `min_gain` interact

Think of `max_iterations` as the planner's **search budget** and `min_gain` as the
**minimum information threshold** for an NBV destination. They affect different parts of the
decision:

- `max_iterations` is an upper bound on RRT* samples. After the initial 1500 iterations,
  `rrt_star()` checks `enough()` every 500 iterations, before taking that iteration's sample.
  In NBV mode, a successful check means at least one current tree leaf sees `min_gain` or
  more frontier cells. If so, tree growth stops; otherwise it can continue, at most to
  8000 iterations.
- `min_gain` is only used by the NBV branch. The `gain()` function counts frontier cells
  within `info_radius` of each leaf, after filtering cells close to remembered visited/stalled
  positions. The leaf must meet the threshold to satisfy the early-stop test and to be
  eligible for scoring.
- For a clicked Task 2 goal, `min_gain` is irrelevant. The early-stop test instead asks
  whether any tree node is within 0.15 m of the clicked goal. More iterations can help find
  that goal, but increasing `min_gain` cannot.
- A larger search budget gives the random tree more chances to reach useful space, but costs
  more computation and does not guarantee a useful viewpoint. If a leaf still cannot see
  enough frontier cells, the NBV cycle returns without a goal.
- A higher `min_gain` rejects small frontier fragments that may be noise or hidden behind
  obstacles, but can also reject real, small remaining frontiers. A lower value accepts
  smaller fragments, but can lead to repeated visits or circling.
- The two settings are not interchangeable: more iterations may discover a qualifying leaf
  sooner in the maze, but cannot make an intrinsically low-gain frontier meet the threshold.
  Conversely, lowering the threshold may allow the planner to stop growing earlier, but
  does not ensure that the resulting goal is reachable by the follower in practice.

Current source defaults: `iterations = 1500`, `max_iterations = 8000`, `min_gain = 7`.
The early-stop callback is checked at iteration indices 1500, 2000, ..., 7500. If no check
passes, the loop completes 8000 iterations. Parameters are read at node startup, so test a
change by editing the default and restarting the node; a live `ros2 param set` does not alter
the values already copied into `self.p`.

### `branch`: recover a route

`branch(nodes, parent, i)` starts at selected node `i`, repeatedly follows its parent index
until `-1` (the root), then reverses the collected points. This converts the tree's
backward parent links into a forward root-to-goal path.

### Planner outline

`NavigationNode` stores the latest map, frontier map and optional RViz goal in callbacks.
Its 1-second `tick()` timer reads the robot pose from TF, handles the current goal state,
and calls `plan()` when a new plan is needed.

`plan()` wraps `/map` as a `Grid`, makes a free-space mask with `free_mask()`, publishes a
visualization of the mask, and roots the tree at the robot (or the nearest free cell if the
robot's cell is not in the mask). It converts `/frontiers` cells to world positions and
filters frontier cells near remembered visited/stalled positions.

`/inflated_map` is the inverse-style visualization: 100 marks known cells excluded from the
planner's usable free-space mask (occupied cells and the clearance margin around them); 0
marks other cells. Do not read 100 as "safe" or as a direct encoding of the free-space mask.

- With a clicked RViz goal, `rrt_star()` grows until it finds a node within 0.15 m of the
  clicked point. The planner selects the found node with the lowest stored path cost.
- Otherwise, it grows one RRT* tree and scores its leaves by
  `H = path_cost - info_weight * frontier_gain + turn_cost * first_turn`.
  A leaf must see at least `min_gain` frontier cells to be considered. Current source sets
  `min_gain` to 7.
- `branch()` follows parent indices back from the selected node to the tree root.
  `publish()` densifies that branch into `/path` and publishes the tree and goal markers for
  RViz.

The following walkthrough will trace these functions and their decisions in source order.
Then it will cover `frontier_detector_node.py`, `path_follower_node.py`, SLAM configuration
and the launch chain. Do not treat the older planner descriptions in other STUDY pages as
current; check the live source above.

## Supporting nodes and startup chain

### `frontier_detector_node.py`

The detector defines a frontier as a cell that is **known free (0 through 50)** and has at
least one **unknown (-1) cardinal neighbor**: up, down, left or right. Occupied neighbors are
not part of the definition. The frontier cell itself is free; it marks the known/unknown
boundary where exploration may reveal more.

`map_callback()` ignores a zero-sized map, reshapes the flat occupancy data into
`(height, width)`, constructs free and unknown masks, shifts the unknown mask in four
directions to find free cells adjacent to unknown, and publishes an `OccupancyGrid` on
`/frontiers`. Its metadata/header come from `/map`; frontier cells are 100 and the rest 0.

### `path_follower_node.py`

- `path_callback()` replaces the stored waypoint list whenever a new `Path` arrives.
- `scan_callback()` stores the newest `LaserScan` and its local receive time.
- `update_robot_pos()` looks up `map → base_link`, converts quaternion orientation to yaw,
  and returns false if TF is unavailable.
- `follow_path()` runs every 0.1 s. It publishes zero if TF is missing, drops waypoints that
  are within `look_ahead` while retaining the final point, stops inside `goal_tolerance`,
  computes heading error and proportional turn command, and turns in place when the error
  exceeds 0.3 rad.
- `forward_clearance()` filters invalid/too-near/too-far laser readings, projects valid
  returns into x/y, and finds the nearest return in the narrow forward strip. A stale/missing
  scan gives no clearance, so forward speed is set to zero. Between `stop_distance` and
  `slow_distance` the speed is scaled down linearly. If still blocked while correctly aimed,
  the follower may reverse a short distance if the rear strip is clear.
- `main()` disables rclpy's default signal handler so the `finally` block can call `stop()`
  and publish zero before destroying the node.

The follower is a differential-drive controller: the command is forward linear speed and
angular speed, not a sideways velocity. The planner detects arrival at its own goal on its
1-second timer; the follower only knows that it reached the final waypoint of its current
path.

#### ELI5 control-loop walkthrough

Think of the path as a row of breadcrumbs. Every 0.1 s, the follower asks: "Where am I?
Which breadcrumb should I aim for? Am I pointed toward it? Is the space immediately in
front clear enough to move?"

1. If there is no path, `follow_path()` returns. If there is a path, it starts a
   `TwistStamped` command with zero linear and angular speed.
2. It asks TF for the robot's map position and yaw. If TF is missing, it publishes that
   zero command and returns, avoiding leaving an old motion command active.
3. While the first waypoint is within `look_ahead` and it is not the last waypoint, it
   removes that waypoint. It keeps the last one so it can detect the end of the route.
4. If that final waypoint is within `goal_tolerance`, it publishes the still-zero command:
   the follower has reached the end of this path.
5. Otherwise it computes distance and direction to the first remaining waypoint.
   - Farther than `look_ahead`: request `max_v`.
   - Closer than `look_ahead`: request `kp_vel * distance`, capped at `max_v`, so it slows
     as it approaches.
   - Heading error above 0.3 radians: set forward speed to zero and turn toward the point.
     The angular command is `kp_yaw * error`, capped at `max_w`.
6. It checks for an active back-off. If the rear scan is fresh and clear and the back-off
   timer has not expired, it publishes a small negative speed and returns. Otherwise it
   cancels the back-off and continues to the forward-scan check.
7. `forward_clearance()` filters invalid laser returns, projects each valid beam into x/y,
   and finds the nearest x-distance among points in a narrow strip in front. With no fresh
   scan it returns `None`; with no obstacle in the strip it returns infinity.
8. No fresh scan means forward speed becomes zero. With a fresh scan, the follower computes
   `gap = (clearance - stop_distance) / (slow_distance - stop_distance)`, clamps it to
   0..1, and multiplies the requested forward speed by that value. For the defaults,
   clearance 0.24 m gives `(0.24 - 0.18) / (0.30 - 0.18) = 0.5`.
9. If the robot is aimed forward but clearance stays at or below `stop_distance + 0.02 m`
   for `blocked_time`, it schedules a short reverse motion. The rear strip must be clear
   before the reverse command is actually sent.
10. It publishes the resulting linear and angular speeds. Linear speed is safety-scaled;
    angular speed is separately limited by `max_w`.

The wall check is a **thin forward strip**, not a full 360-degree collision planner. It
doesn't change the path or pick another goal. It just slows/stops the follower, with a brief
back-off attempt when blocked. The map inflation in the planner is the other safety layer.

### SLAM configuration and launch

`ros2_ws/src/r7021e_exploration/config/slam_config.yaml` configures `slam_toolbox` frames
(`map`, `odom`, `base_link`), scan topic, mapping mode, map resolution/update timing and
scan-matching/loop-closure settings. In `exploration.launch.py`, the local `config` path is
built from the package share directory and this YAML is passed in the lifecycle node's
`parameters` list. The launch file also declares a `slam_params_file` argument, but the
current code does not use that substitution: it passes the hardcoded package `config` path.
Do not expect overriding `slam_params_file:=...` to change the file actually loaded.

The YAML supplies SLAM settings; it does not itself create the map. `slam_toolbox` uses those
settings with incoming sensor/odometry data to build and publish the occupancy map. Gazebo
must use simulation time consistently (`use_sim_time:=true`); on the real robot it stays
false. SLAM publishes `/map` and the `map → odom` correction, while wheel odometry supplies
the smoother but drifting `odom → base_link` transform.

`exploration.launch.py` declares `use_sim_time` (false by default) and `task1` (false by
default). It starts the frontier detector, either navigation or Task 1 baseline, the path
follower, and RViz. It also starts `slam_toolbox` as a lifecycle node and emits configure and
activate transitions when autostart is enabled and a lifecycle manager is not being used.
`task1:=true` swaps out the planner, not the follower. `maze_world.launch.py` is a separate
Gazebo launch and is not required on the real robot.

### System chain to say aloud

`/scan` plus odometry/TF feed SLAM → SLAM publishes `/map` and `map → odom` → the detector
publishes `/frontiers` → navigation reads both maps and TF and publishes `/path` →
the follower combines `/path`, TF and `/scan` and publishes `TwistStamped` `/cmd_vel` →
the robot or Gazebo bridge moves the base. RViz visualizes map/path/tree/goal and can send
`/goal_pose` for the Task 2 known-goal test.

## Demo checklist and command order

Use [`../RUNSHEET.md`](../RUNSHEET.md) as the operational source. This short list follows the
Lab Instructions task order. ROS commands below are for Dominic to type in his own terminals;
do not run them from this session.

### Before the demo: real-robot setup

First confirm the TurtleBot number with the TA/label on the robot. For turtle N, the IP is
`192.168.50.N0` and the ROS domain is `3N`. Replace N with the confirmed digit; do not type
the literal placeholders `N`, `3N`, or `N0`. For example, turtle 4 means IP
`192.168.50.40` and domain `34`. Do not trust the stale `.bashrc` mapping without checking
the actual robot. The robot's SSH terminal does not need the laptop setup lines. Every
laptop ROS terminal needs these, using the confirmed domain:

```bash
export TURTLEBOT3_MODEL=burger
```

```bash
export ROS_DOMAIN_ID=3N
```

```bash
source ~/R7021E-group6/lab3/ros2_ws/install/setup.bash
```

Keep a separate laptop terminal ready with the stop command. Type it only if a stop is
needed:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{}"
```

Start robot bringup over SSH and leave it running:

```bash
ssh turtle@192.168.50.N0
```

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

On the laptop, verify the ROS graph and command message type before running the lab nodes:

```bash
ros2 topic list
```

```bash
ros2 topic info /cmd_vel -v
```

Expected: `/scan`, `/odom`, `/tf`, `/cmd_vel`; `/cmd_vel` uses `geometry_msgs/msg/TwistStamped`
and initially has the robot's subscriber. Then prove motion without our code, and stop it:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.05}}}"
```

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{}"
```

Before launching on the real robot, compare laptop and robot clocks. If they differ, follow
the run sheet's clock-sync step; time mismatch can make TF lookups fail even when the graph
and topics otherwise look healthy.

```bash
date +%T
```

```bash
ssh turtle@192.168.50.N0 date +%T
```

If the clocks differ, and the TA permits correcting the TurtleBot clock, use the run sheet's
sync command from the laptop and enter the robot's sudo password:

```bash
ssh -t turtle@192.168.50.N0 "sudo date -u -s '$(date -u +%Y-%m-%dT%H:%M:%S)'"
```

If recording a run, start `ros2 bag record` in its own laptop terminal before launching the
stack. Give each run a new bag name; stop the launch first, then stop the recorder once and
wait for `Recording stopped`.

```bash
ros2 bag record -o ~/bags/<unique-run> /map /frontiers /path /cmd_vel /odom /tf /tf_static /scan /rrt_tree /goal_marker /inflated_map
```

### Gazebo alternative for repeatable comparisons

This is the safer place to repeat Tasks 3 and 4. Use two terminals, with the laptop start
lines in each. First start the small maze:

```bash
ros2 launch ~/R7021E-group6/lab3/sim/maze_world.launch.py world:=lab3_maze_small
```

Then start the stack with simulation time:

```bash
ros2 launch r7021e_exploration exploration.launch.py use_sim_time:=true
```

For Task 1 in this world, start Gazebo with the robot one cell back so it has room to drive
the 0.5 m baseline leg:

```bash
ros2 launch ~/R7021E-group6/lab3/sim/maze_world.launch.py world:=lab3_maze_small x_pose:=-0.8
```

### Task 1: baseline path

Check that there is at least 0.5 m of clear floor in front of and behind the robot. Start the
package launch with Task 1 selected:

```bash
ros2 launch r7021e_exploration exploration.launch.py task1:=true
```

For Gazebo, add `use_sim_time:=true` to that launch command. In the small maze, start the
Gazebo robot at `x_pose:=-0.8` as shown above so the 0.5 m baseline leg has room.

Say what to look for: `leg 1`, `leg 2`, and repeated 0.5 m forward/return paths. The baseline
publishes `/path`; the follower converts it to `/cmd_vel`. Stop the launch after showing the
baseline.

### Task 2: plan to a known goal

Start the normal stack (no `task1:=true`):

```bash
ros2 launch r7021e_exploration exploration.launch.py
```

In RViz, wait for a visible map, choose **2D Goal Pose**, and click a known free point. The
direction arrow is ignored. Explain that the node grows RRT* from the robot and looks for a
tree node within 0.15 m of the clicked point; it publishes the lowest-stored-cost qualifying
branch. Show the goal log, tree and path. This run also starts automatic NBV exploration
after the clicked-goal request is handled.

### Task 3: collision avoidance (optional in the Lab Instructions)

The instructions ask for a collision-avoidance method; this implementation uses map
inflation, and the follower additionally uses a laser stop/slow-down. For a comparison,
Dominic changes `inflation_radius` in `navigation_node.py` to `0.0`, restarts and observes the path;
then restore `0.105` and restart for the padded path. No rebuild is needed with the existing
symlink install. Record what you actually observe; do not claim a wall contact unless it
happens in that run.

### Task 4: exploration gain

For the optional/planned comparison, Dominic changes `info_weight` in `navigation_node.py`
to `0.0`, `0.10`, and `0.30` for separate fresh explorations, restarting the stack between
runs. For a fair comparison in Gazebo, restart the maze world too so each run begins from the
same robot pose and a fresh map. Explain the
score `H = L - info_weight * I + turn_cost * turn`: L is branch length, I is frontier-cell
gain within `info_radius`, and turn is the first heading change. The code requires at least
`min_gain` frontier cells (default 7) before a leaf can be selected. Compare recorded time
to 90% mapped area, total time, and goal order only if you have completed the three runs;
one run per value is not a robust statistical comparison.

### Task 5: complete autonomous exploration

Use the normal launch with no Task 1 switch. The timer repeatedly plans, publishes `/path`,
waits for arrival/stall/path age, and plans again. It finishes after ten consecutive NBV
cycles without an eligible leaf, after the first plan. Show the map, path, RRT tree, goal
marker, planner log and bag/screen recording. Report observed results, not a predicted
completion.

The Lab Instructions also ask for a short report (about three pages with figures), focused
on Task 4: node inputs/outputs/workflow, planner pseudocode, the exploration method and its
information-gain equation, and problems encountered during real-time operation. They ask
for a robot video and an RViz screen recording. Keep real-robot observations separate from
Gazebo measurements.

### What is and is not demonstrated yet

The current committed run artifacts include one full Gazebo run of the small maze and two
big-maze NBV runs, one of which was cut short. They do not show a controlled Task 4 comparison
at all three weights or a Task 3 comparison at both inflation settings. The run sheet marks
the real TurtleBot procedure as not tested. In a one-hour rehearsal, prioritize robot/domain
preflight, Task 1, a known-goal click, then the full run if time and the TA allow. Use Gazebo
for repeated parameter comparisons; do not present Gazebo results as real-robot observations.
