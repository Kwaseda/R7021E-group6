# Lab 3 code, file by file

This is the simple version (2026-10-11). The older frontier-cluster planner is in `../lab3-old`.
Only `navigation_node.py` is new. The other files are the same as in `lab3-old`.

## How the parts meet

```
turtlebot3 (robot or Gazebo) --/scan, /odom, /tf--> slam_toolbox --/map, map->odom--+
                                                                                    |
frontier_detector_node <---------------------------------------- /map -------------+
        | /frontiers                                                                |
        v                                                                           v
navigation_node (OURS: RRT* + next best view) --/path--> path_follower_node --/cmd_vel--> robot
        | /rrt_tree, /goal_marker, /inflated_map (RViz and the bag)
```

Every node is one file. Callbacks only store messages. A timer does the work.

---

## `navigation_node.py`: where to go next and how to get there (Tasks 2, 3, 4, 5)

### The idea in four sentences

Every second, if the robot has no goal, the node grows one RRT* tree from the robot over the
known free space, with walls inflated by the robot radius. Each leaf of the tree is a candidate
view point. A leaf scores `H = L - w*I + c*turn`, where L is the branch length, I is the number
of frontier cells within 0.75 m of the leaf, and turn is the first heading change. The branch to
the lowest-H leaf is the path, and the node sends it to the follower and waits.

This is next-best-view (NBV): the tree itself gives the candidates. There is no frontier
clustering and no separate goal search.

### Values (`DEFAULTS`, top of the file)

| Name | Value | Why |
|---|---|---|
| `inflation_radius` | 0.105 m | Burger radius. Rounded up to 3 cells, so a 0.15 m collar. Task 3 |
| `iterations` | 1500 | Enough to cover a small maze. The tree stops here if a leaf already sees a frontier |
| `max_iterations` | 8000 | Keeps growing in steps of 500 when no leaf sees a frontier yet, for far frontiers in a big maze |
| `step_size` | 0.30 m | Shorter than half a 0.7 m corridor |
| `rewire_radius` | 0.60 m | Twice the step, so a new node has neighbours to rewire |
| `info_radius` | 0.75 m | The PDF tip: a short laser range for the gain makes the robot explore near first |
| `info_weight` | 0.10 m per cell | One frontier cell is worth 10 cm of driving. 0 makes it go to the nearest leaf that sees anything |
| `turn_cost` | 0.15 m per rad | Prefers going on over turning back. A half turn costs as much as 0.47 m of driving |
| `arrival_tolerance` | 0.20 m | Arrived. The follower stops within 0.05 m of the last point, so this is safe |
| `stall_timeout` | 8 s | Less than 0.05 m of progress for 8 s means stuck. Plan again |
| `max_path_age` | 20 s | The map grows while driving. After 20 s a fresh tree is better |
| `max_empty_cycles` | 10 | 10 plans in a row with no leaf seeing a frontier means done. Counted only after the first plan |
| `seen_radius` | 0.50 m | A frontier cell still there after the robot stood within 0.5 m of it cannot be seen from free space. It stops counting. Without this, the robot crept 0.2 m at a time toward frontier behind a wall (run nbv-small-1) |

### Functions

| Function | What it does | Why |
|---|---|---|
| `yaw_of(q)` | Quaternion to planar yaw: `atan2(2(wz+xy), 1-2(y²+z²))` | TF gives orientation as a quaternion |
| `Grid` | Wraps an OccupancyGrid as a numpy array. `cell(x,y)` gives row and column with `floor`. `world(r,c)` gives the cell centre | Row is y, column is x. `floor` is right for negative coordinates, `int()` is not |
| `free_mask(grid, radius)` | Marks occupied cells (>50), grows them by a disc of k = ceil(radius/res) cells, returns known-free and not-grown | Task 3. The tree treats the robot as a point. Unknown (-1) is not free, so the tree never plans into the unknown |
| `segment_free(mask, grid, a, b)` | Checks cells along a line every half cell | The collision check for every tree edge |
| `rrt_star(mask, grid, root, prm, rng, enough)` | Samples a random free cell, finds the nearest node, steps 0.30 m toward the sample, keeps the new node if the edge is free, picks the cheapest parent within 0.60 m, rewires neighbours that get cheaper through it. Every 500 samples after `iterations`, it asks `enough()` whether to stop | Task 2. Sampling only free cells wastes no samples on walls or unknown. The tree can only grow through free edges, so every node is reachable from the robot |
| `branch(nodes, parent, i)` | Follows parents from node i back to the root, reversed | The path to a leaf |
| `NavigationNode.__init__` | Reads `DEFAULTS` as ROS parameters. Publishers `/path`, `/rrt_tree`, `/goal_marker`, `/inflated_map`. Subscribers `/map`, `/frontiers`, `/goal_pose`. TF listener. 1 s timer | Template shape |
| `on_map`, `on_frontiers` | Store the latest message | Callbacks store, the timer computes |
| `on_click` | Stores a goal from RViz "2D Goal Pose" | Task 2 test with a known goal |
| `pose()` | TF lookup `map -> base_link`, latest. Returns (x, y, yaw) or None | SLAM corrects odometry drift through `map -> odom` |
| `tick()` | With a goal: arrived, or progress (reset the stall clock, keep driving until the path is 20 s old), or stalled. Arrived and stalled spots go into `visited`. Without a goal: `plan()` | Task 5 loop: decide, plan, send, wait for the end |
| `plan(pose, now)` | Builds the mask and publishes it. Roots the tree at the robot, or at the nearest free cell if the robot stands in the collar. Clicked goal: grows until a node is within 0.15 m of it and takes the cheapest such node ("shortest branch to the goal", Task 2). Otherwise NBV: scores the leaves, keeps the lowest H, prints one line | Tasks 2 and 4 |
| `gain(nodes)` (inside `plan`) | For each node, counts frontier cells within `info_radius` | The information gain I |
| `leaves_of(parent)` | Nodes with no children | NBV evaluates the edge nodes of the tree, as the PDF says |
| `finish(pose)` | Sends a one-point path at the robot, prints `exploration finished`, stops planning | The follower holds the robot still |
| `publish(points, nodes, parent)` | Path with a point every 0.10 m, plus the tree as a LINE_LIST and the goal as a red sphere | The follower drops points within `look_ahead`. Dense points stop it from cutting corners |
| `publish_inflated(grid, mask)` | 100 where known and not usable, 0 elsewhere | Task 3 evidence in RViz and in the bag |
| `main()` | init, spin, shut down | |

### The log line

```
t=82.0 goal=(0.89, 1.06) H=-4.00 L=0.57 I=46 turn=0.19 nodes=1500 known=11.2m2 plan_time=0.25s
```

`nodes` is the tree size. `known` is the mapped area, and it is the 90 % measure for the competition.

---

## `path_follower_node.py`: given, with fixes from Gazebo runs

Template behaviour: pops waypoints within `look_ahead`, steers to the next one, turns on the
spot when the heading error is above 0.3 rad. Our additions, each with the run that showed it
(details in `../lab3-old/LOG.md`):

| Addition | Why |
|---|---|
| `look_ahead` 0.12 (was 0.2) | 0.2 aimed past corners into walls |
| `kp_yaw` 1.0, `max_w` 0.6 | Fast turns slipped in Gazebo, so the SLAM pose drifted |
| `forward_clearance()` and the slow-down from 0.30 to 0.18 m | Reactive stop that uses the laser directly. With it off, the robot hit a wall in the big maze when SLAM was 2.6 m wrong |
| back-off 0.10 m after 2 s blocked | Without it the robot froze at corners |
| blocked also within 0.02 m above the stop distance | The slow-down only nears the stop distance, so the robot crept at 0 m/s forever |
| stop at the end of the path (`goal_tolerance` 0.05) | The template never stopped |
| zero when TF fails, `stop()` on Ctrl-C, `SignalHandlerOptions.NO` | `turtlebot3_node` keeps the last command. rclpy's own handler closes the context before the stop can go out. Proved: 0.15 m/s on after Ctrl-C before, 0.000 after |
| scan age measured on arrival | The robot clock was 325 days off in Lab 1 |

## `frontier_detector_node.py`: given, not changed

A free cell (0 to 50) with an unknown 4-neighbour is a frontier, value 100, on `/frontiers`.

## `task1_baseline.py`: Task 1

Timer 0.5 s. On the first pose it sends a 0.5 m straight path ahead. When the robot is within
0.10 m of the end, it sends the reverse path, and repeats. `leg N` in the log.

## `launch/exploration.launch.py`

Starts the frontier detector, the navigation node (or `task1_baseline` with `task1:=true`), the
follower, RViz and slam_toolbox (a lifecycle node, configured and then activated by events).
`use_sim_time:=true` for Gazebo. The two arguments are declared first, because a default only
exists after its declaration runs.

## `config/slam_config.yaml`

Template values, except: `map_update_interval` 1.0 s, `minimum_time_interval` 0.1,
`minimum_travel_distance` 0.2, `minimum_travel_heading` 0.3 (a fresh map often enough to plan
on), and `do_loop_closing` false (two false closures in the repetitive maze: pose error 0.15 to
1.7 m in 4 s). This is the only YAML: slam_toolbox needs it, the course gave it.

## Outside the package

- `sim/maze_world.launch.py` and `sim/worlds/`: Gazebo mazes. Not graded.
- `video/bag_to_video.py`: makes the demo video from a bag.

## Pseudocode for the report

```
every 1 s:
    if goal: if arrived or stalled: remember spot, goal = none
             elif moving and path younger than 20 s: continue
    mask = known free minus walls grown by 0.105 m
    tree = RRT*(root = robot, samples from mask)            # grow until a leaf sees frontier
    frontier = frontier cells not within 0.5 m of a remembered spot
    for each leaf: I = frontier cells within 0.75 m
                   H = cost(leaf) - 0.10*I + 0.15*|first turn|
    if no leaf has I > 0: count; after 10 in a row: stop
    else: send branch to best leaf as /path
```
