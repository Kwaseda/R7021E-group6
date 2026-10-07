# Lab 3 theory sheet

Read one module. Close this file. Then answer the questions for that module in `QUESTIONS.md`
out loud, with no notes. Do this in order. Each module takes about 20 minutes to read.

Every module names the function in `navigation_node.py` or `path_follower_node.py` that uses the
idea. When you read the code later, you will see the same words.

Numbers come from the code and from `LOG.md`. A number marked [desk] came from the offline
simulation. It is not a Gazebo number.

---

## T1. Map, frames and TF

**The occupancy grid.** A map is a list of numbers. Each number is one square cell. The message
`nav_msgs/OccupancyGrid` holds `width`, `height`, `resolution` and `origin`. Our map has
resolution 0.05 m, so one cell is 5 cm.

| Value | Meaning | In our code |
|---|---|---|
| -1 | unknown | not free, not a wall |
| 0 to 50 | free | free |
| above 50 | occupied | wall |

The list is row by row. Cell (row, col) has index `row * width + col`. The origin is the world
position of the corner of cell (0, 0). So:

    col = floor((x - origin_x) / resolution)
    row = floor((y - origin_y) / resolution)

A common mistake is to swap row and col. Row follows y. Column follows x. NumPy reshapes the
list to `(height, width)` for this reason. Code: `Grid.__init__`, `Grid.cell`, `Grid.world`.
`Grid.world` adds 0.5 so it returns the centre of the cell.

**Frames.** Three frames matter.

- `map` is fixed to the world. slam_toolbox owns it.
- `odom` is fixed to where the robot started. It is smooth, but it drifts.
- `base_link` moves with the robot.

The wheel odometry publishes `odom` to `base_link`. slam_toolbox publishes `map` to `odom`. This
second transform is the correction. When SLAM closes a loop, it changes `map` to `odom` and
leaves the odometry alone.

The planner needs the robot pose in the map frame, because the map and the goals are in that
frame. So it asks TF for `map` to `base_link`. It does not read `/odom`, because `/odom` has no
correction. Code: `get_robot_pose`.

**SLAM in one paragraph.** slam_toolbox takes laser scans and odometry. It matches each new scan
to the earlier scans and so finds where the robot is. It adds the scan to the map. It keeps a graph
of poses. When the robot sees a place again, the graph finds a loop and pulls the map straight.
We use the async mode. It takes a scan only if the robot moved 0.2 m or turned 0.3 rad, and it
publishes the map every 1.0 s. Lab 4 covers this in detail.

**Simulation time.** Gazebo publishes its own clock on `/clock`. Every node must run with
`use_sim_time:=true`. If one node does not, its time stamps do not match TF, and you see
"extrapolation" and "lookup failed" messages.

---

## T2. Safe space (Task 3)

**The problem.** RRT* plans for a point. The robot is a disc with a radius of about 0.105 m. A
path that runs 2 cm from a wall is fine for a point. It drives the robot into the wall.

**The fix: inflate the walls.** Make every wall cell fatter by the robot radius. The robot centre
may then go anywhere that is not inside the fat walls. Code: `planning_mask`, in four steps.

1. Take the occupied cells (value above 50).
2. Remove single cells that have no occupied neighbour. These are scan noise. The old repo found
   that this mattered more than the radius.
3. Grow the rest by a disc of `k = ceil(0.105 / 0.05) = 3` cells. This is `dilate`. The collar is
   3 cells, 0.15 m.
4. The mask is True where a cell is known free and not inside the grown walls.

Unknown cells are never True. The planner does not drive into space that it has not seen.

**Connected to the robot.** `reachable` is a flood fill from the robot cell. It keeps only free
cells that connect to the robot through 8 neighbours. A goal in a free pocket that the robot
cannot reach is then not a candidate. Without this step, 5 of 12 desk runs stopped at 57 %
coverage [desk].

**The robot is inside the collar.** After a turn near a wall, the robot centre can be inside the
grown wall. The tree then starts from the nearest free cell (`nearest_free`) and the robot
position goes at the front of the path.

**Check the whole edge.** A new tree node can be free while the edge to it crosses a wall corner.
So `segment_free` tests the line every half cell (2.5 cm), which is less than the wall thickness.
It is a checked line, not a checked point.

**Trade-off.** Too little inflation: the robot scrapes walls. Too much: narrow corridors close
and parts of the maze become unreachable. A free corridor must be wider than twice the collar
(0.30 m) plus one cell, so about 0.35 m between wall faces.

**Second layer.** The follower has its own check with the laser. It slows the robot when
something is in the 0.20 m wide strip ahead (see T5). The inflated map is the planned safety.
The laser is the reactive safety. A potential-field method (APF) is a reactive method. It is
simple, but it gets stuck in local minima, so it is not a good main planner.

---

## T3. RRT and RRT* (Task 2)

**RRT, five steps.** Start a tree at the robot. Repeat:

1. Sample a random point.
2. Find the nearest node in the tree.
3. Steer from that node toward the sample by one step.
4. Check that the new point and the edge are free. This is the only step that needs the map.
5. Add the node to the tree, with its parent.

Stop when a node is within the goal tolerance of the goal. Read the path back through the parents.

**Our values.** Step 0.30 m. Goal bias 10 %: one sample in ten is the goal itself, which pulls
the tree toward it. Other samples are random known-free cells with a small random offset inside
the cell. Goal tolerance 0.15 m. Code: `rrt_star`.

- Large step: the tree grows fast but cannot enter tight places.
- Small step: the tree is slow but it follows corridors.
- Goal bias 0: the tree spreads out evenly and finds the goal late.
- Goal bias near 1: the tree runs straight at the goal and gets stuck behind the first wall.

**RRT* adds two things.**

- *Choose the best parent.* The new node does not attach to the nearest node. It attaches to the
  neighbour (within 0.60 m) that gives the lowest cost from the root. The cost is path length.
- *Rewire.* If the route to a neighbour is shorter through the new node, the neighbour changes
  its parent. All its descendants get the new, lower cost (`shift_cost`).

The cost is more work for each iteration. The gain is that the path gets shorter as the number
of iterations grows. RRT gives a path. RRT* gives a better path with more time.

**Our run rules.** After the first hit on the goal, the tree grows 200 more iterations. Then the
node takes the cheapest hit. The budget is 1500 iterations. If there is no path, it tries once more
with 15000. In 25 random pairs in the big maze, 1500 iterations solved 6, 6000 solved 21 and 15000
solved all 25 [desk]. This is why the second budget exists.

**Why sampling, not A* on the grid.** A* is optimal on the grid and has no randomness. But the
work grows with the number of cells, and the path follows grid directions. RRT* samples the space,
so it copes with large maps and gives smooth paths. A* wins on a small known map where you need
the exact shortest path every time. RRT is probabilistically complete: if a path exists, the
chance to find it goes to 1 as the samples grow. It is not optimal. RRT* is asymptotically optimal.

---

## T4. Frontiers and information gain (Task 4)

**Frontier.** A free cell that touches an unknown cell. The detector (given) checks the four
neighbours up, down, left, right. It publishes a map on `/frontiers` where each frontier cell is
100. Frontiers mark the edge of what we know. New information can only come from there.

**From cells to goals.** `cluster_cells` joins frontier cells that touch, with all 8 neighbours,
into clusters. A cluster with fewer than 5 cells is noise and is dropped. For each cluster,
`cluster_goal` takes the mean position. If that point is in the reachable mask, it is the goal.
If not, it takes the reachable cluster cell nearest the robot. The node keeps at most 6 goals,
the nearest by straight line, and skips goals that are closer than 0.20 m to the robot or closer
than 0.30 m to a goal it crossed off.

**The score.** The node plans a path to each goal with RRT*. Then it scores each path.

    H = L  -  w * I  +  c * turn

| Symbol | Meaning | Value |
|---|---|---|
| L | length of the RRT* path in metres | from the path |
| I | frontier cells within r of the goal | r = 0.75 m |
| w | how many metres of travel one frontier cell is worth | 0.10 |
| turn | angle between the robot heading and the first path segment | rad |
| c | metres of cost per radian | 0.15 |

The lowest H wins. Code: `score`.

**What I really measures.** The lab says to count unknown cells in a reduced sensor range. Our I
counts frontier cells in a 0.75 m disc. This is a proxy: a place with many frontier cells next
to it has a lot of unseen space behind it. It is cheaper than a ray cast and it is what the old
repo tuned. It is not the same number as the unknown area. Be ready to say this.

**Why a short radius.** The laser sees 3.5 m. At that range from a goal, almost every
frontier is in view of almost every goal. All goals then score the same I and the robot cannot
choose. A short radius makes I local.

**The weight w.**

- w = 0: only L and turn count. The robot goes to the nearest frontier by path length. This is
  greedy. Expected: fast first minutes, and some driving back and forth later.
- w very large: only I counts. The robot goes to the richest frontier, however far. Expected: it
  wastes time driving.
- In between: the trade-off. The report must justify the value.

**When to stop.** After the first goal, if the node finds no goal for 10 cycles in a row (10 s),
it stops. It sends a one-point path at the robot position, so the follower holds still.

---

## T5. Following the path (given follower, plus our changes)

**Differential drive.** The robot has two wheels. The commands are forward speed `v` and turn
rate `w`. It cannot move sideways. To go to a point, it turns toward it, then drives.

**P control.** The follower is a proportional controller. Code: `follow_path`.

1. Pop every waypoint that is closer than `look_ahead` (0.2 m). The first waypoint that is
   farther is the target.
2. Compute the heading error to the target.
3. Turn rate: `w = kp_yaw * error` (kp_yaw 2.0), cut at 1.0 rad/s.
4. Forward speed: `max_v` (0.15 m/s), or `kp_vel * distance` when the target is nearer than
   `look_ahead`.
5. If the heading error is above 0.3 rad, set the forward speed to 0. The robot turns on the
   spot, then drives. Without this, a big error makes it arc away from the path.

The path waypoints are 0.10 m apart. The follower pops points inside 0.2 m. A sparse path
would make it cut corners.

**Our three changes.**

- *Scan slow-down.* A strip ahead of the robot, 0.20 m wide (half width 0.10 m). The nearest
  return in the strip sets the speed factor: 0 at 0.18 m, 1 at 0.30 m, linear between. A scan
  older than 0.5 s counts as missing, and the speed is 0.
- *Stop at the end.* Within 0.05 m of the last waypoint the follower publishes zero speed.
- *Guarded TF.* If TF fails, it sends nothing and tries again.

**Known gap.** The laser ignores returns closer than 0.12 m (`range_min`). Something at 0.10 m
ahead is invisible. The robot stops at 0.18 m, so it should not get there. The laser is also
3 cm behind the robot centre, and the code ignores this.

**The follower does not know that the robot arrived.** The planner finds out: it measures the
distance to its own goal each second (T6).

---

## T6. The loop and how it fails (Task 5)

**One cycle.** A timer fires every 1.0 s. Code: `_tick`.

1. If there is no map or no frontier map yet, wait.
2. Read the robot pose from TF.
3. If there is a goal:
   - closer than 0.20 m: *arrived*. Cross the goal off.
   - moved less than 0.05 m in 8 s: *stalled*. Cross the goal off.
   - path younger than 20 s: wait for the next tick.
   - path older than 20 s: plan again.
4. If there is no goal, call `plan_path`. If it returns a path, publish it. If not, count an
   empty cycle. Ten empty cycles in a row (after the first goal) end the run.

Why a timer and not the map callback? A plan can take seconds. The map arrives every second.
The arrival, stall and age rules need a steady clock.

**Failure cases.**

| Case | What the robot does now |
|---|---|
| RRT* finds no path | Crosses that goal off and tries the next candidate |
| Robot stuck against something | Stall rule crosses the goal off after 8 s |
| No frontier cells | Returns None. Ten cycles later the run ends |
| Frontier next to a wall | The cluster middle is not in the mask, so it takes the nearest reachable cluster cell |
| Map changes while RRT* runs | The plan uses a copy of the map from the start of the cycle. A path through a newly found wall is not noticed until the robot arrives, stalls, or the path is 20 s old |

**Weak points we know** (from `LOG.md`).

- A robot in a closed pocket of the map ends the run after 10 empty cycles, even if frontiers
  remain somewhere it cannot reach.
- A goal that was crossed off stays crossed off within 0.30 m, even if the map later opens a way.
- The first plan on a real map can take a few seconds. The robot follows the old path meanwhile.
- Corner touches are expected. The gap between the collar and the robot radius is small.
