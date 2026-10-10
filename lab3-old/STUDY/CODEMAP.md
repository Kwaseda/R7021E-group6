# Lab 3 code map

Use this file to learn how the nodes work and how they connect. Read the picture first. Then
close the file and draw it from memory. Then open the code in the order at the end.

---

## The picture

```
                       /scan  (LaserScan)
         +-------------------------------------+
         |                                     |
   robot / Gazebo                              v
   (turtlebot3)  --- TF odom->base ----------> slam_toolbox ---> /map  (OccupancyGrid)
         ^                                     |                   |
         |                                     +-- TF map->odom    |
         |                                                         v
         |                                              frontier_detector_node
         |                                                         |
         |                                                    /frontiers
         |                                                         |
         |     RViz "2D Goal Pose" ---- /goal_pose ----+           v
         |                                             +--> navigation_node  (you read this)
         |           TF map->base_link ----------------+--> publishes:
         |                                                   /path  (nav_msgs/Path)
         |                                                   /rrt_tree, /goal_marker  (RViz only)
         |                                                         |
         |                                                         v
         +<------------- /cmd_vel (TwistStamped, 10 Hz) --- path_follower_node
                                                                   ^
                                       /scan (slow-down) ---------+
                                       TF map->base_link ---------+
```

With `task1:=true`, `task1_baseline` takes the place of `navigation_node`. It publishes `/path` too.

Three nodes decide three things.

- `navigation_node` decides *where* to go and *how* to get there (the path).
- `path_follower_node` decides *how to drive* along the path.
- `frontier_detector_node` finds the edges of the known map. It decides nothing.

---

## Nodes

| Node name with `ros2 launch` | Name with `ros2 run` | File | Source |
|---|---|---|---|
| `/frontier_detector` | `/frontier_detector` | `frontier_detector_node.py` | given, not changed |
| `/navigation_node` | `/path_planner_node` | `navigation_node.py` | ours, 614 lines |
| `/path_follower_node` | `/path_follower` | `path_follower_node.py` | given, three additions |
| `/task1_baseline` | `/task1_baseline` | `task1_baseline.py` | ours, 88 lines |
| `/slam_toolbox` | | slam_toolbox, async mode | given, four values changed |
| `/rviz` | | rviz2 | given, two displays added |

The launch file sets the name. So `ros2 param get /navigation_node info_weight` works under
`ros2 launch`. Under `ros2 run` the name is `/path_planner_node`. Check with `ros2 node list`.

---

## Topics

| Topic | Type | From | To | Rate |
|---|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | robot or Gazebo | slam_toolbox, path_follower | about 5 Hz on the robot |
| `/odom` | `nav_msgs/Odometry` | robot or Gazebo | none of our nodes. slam_toolbox reads the TF `odom` to `base_link` | |
| `/map` | `nav_msgs/OccupancyGrid` | slam_toolbox | frontier_detector, navigation_node | every 1.0 s |
| `/frontiers` | `nav_msgs/OccupancyGrid` | frontier_detector | navigation_node | once per map |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz tool | navigation_node | on a click |
| `/path` | `nav_msgs/Path` | navigation_node or task1_baseline | path_follower | on each new plan |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | path_follower | robot or Gazebo | 10 Hz |
| `/rrt_tree` | `visualization_msgs/Marker` | navigation_node | RViz | on each new plan |
| `/goal_marker` | `visualization_msgs/Marker` | navigation_node | RViz | on each new plan |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | robot, slam_toolbox, robot_state_publisher | all | |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo | all, with `use_sim_time` | |

Notes you can use at the oral:

- `/cmd_vel` is `TwistStamped`, not `Twist`, in ROS 2 Jazzy for the TurtleBot3. If a sender uses
  `Twist`, the robot does not move and there is no error. Check with `ros2 topic info /cmd_vel -v`.
- The follower reads `/scan` with the sensor-data QoS (best effort). A reliable subscription to a
  best-effort publisher can give no data. The reverse is fine.
- The planner reads the map and the frontier map. It never reads `/scan` or `/odom`.
- The detector publishes `/frontiers`. The template default for the planner was `frontier`. We
  changed it, because a wrong topic name gives no error.

## Transforms

```
map  --(slam_toolbox)-->  odom  --(wheel odometry)-->  base_link
```

On the Burger the last step goes through `base_footprint`. Check with
`ros2 run tf2_tools view_frames`. The planner and the follower both look up `map` to `base_link`.

---

## Parameters

All planner values are in the `DEFAULTS` dictionary at the top of `navigation_node.py`. Each is a
ROS parameter. The node reads them once, at start. `ros2 param set` after the start changes nothing.
To change a value: edit `DEFAULTS`, save, and restart the launch. The build uses `--symlink-install`,
so a rebuild is not needed.

| Group | Name | Value | What it changes | Task |
|---|---|---|---|---|
| goal handling | `replan_period` | 1.0 s | how often `_tick` runs | 5 |
| | `arrival_tolerance` | 0.20 m | goal reached | 5 |
| | `stall_distance`, `stall_timeout` | 0.05 m, 8 s | the robot did not move enough | 5 |
| | `max_path_age` | 20 s | plan again after this age | 5 |
| | `max_empty_cycles` | 10 | cycles with no goal before the run ends | 5 |
| | `retire_radius` | 0.30 m | no new goal this close to a crossed-off goal | 5 |
| map | `inflation_radius` | 0.105 m | collar, rounded up to 3 cells | 3 |
| | `occupied_threshold` | 50 | value above this is a wall | 3 |
| | `waypoint_spacing` | 0.10 m | distance between path points | 1, 5 |
| RRT* | `max_iterations`, `retry_iterations` | 1500, 15000 | search budget | 2 |
| | `step_size` | 0.30 m | one steer | 2 |
| | `goal_bias` | 0.10 | share of samples that equal the goal | 2 |
| | `goal_tolerance` | 0.15 m | a node this close to the goal counts as a hit | 2 |
| | `rewire_radius` | 0.60 m | neighbours for the parent choice and rewire. Keep it above `step_size` | 2 |
| | `extra_iterations` | 200 | growth after the first hit | 2 |
| exploration | `min_cluster_size` | 5 cells | smaller clusters are noise | 4 |
| | `max_candidates` | 6 | goals planned in each cycle | 4 |
| | `info_radius` | 0.75 m | the disc for I | 4 |
| | `info_weight` | 0.10 | w in H | 4 |
| | `turn_cost` | 0.15 | c in H | 4 |

Follower (declared in `path_follower_node.py`): `max_v` 0.15, `max_w` 1.0, `kp_vel` 1.0,
`kp_yaw` 2.0, `look_ahead` 0.2, `stop_distance` 0.18, `slow_distance` 0.30, `half_width` 0.10,
`scan_timeout` 0.5, `goal_tolerance` 0.05.

Task 1: `task1_baseline` has `length` 0.5 m and `tolerance` 0.10 m.

SLAM (`config/slam_config.yaml`): `map_update_interval` 1.0, `minimum_time_interval` 0.1,
`minimum_travel_distance` 0.2, `minimum_travel_heading` 0.3, `do_loop_closing` true.

---

## The launch file

`exploration.launch.py` starts, in this order:

1. Two launch arguments: `use_sim_time` (default false) and `task1` (default false). They must be
   declared first, because the nodes read them when they start.
2. `frontier_detector`, then `navigation_node` (only if `task1` is false), then `task1_baseline`
   (only if `task1` is true), then `path_follower_node`, then RViz.
3. `slam_toolbox` as a lifecycle node. The launch file configures it, then activates it.
   Until it is active, there is no `/map` and no `map` to `odom` transform.

Every node gets `use_sim_time`. For Gazebo, set `use_sim_time:=true`. For the real robot, leave it out.

---

## One planning cycle, as calls

```
_tick                                   every 1.0 s
 |- get_robot_pose                      TF map -> base_link
 |- (goal set?) arrived? stalled? too old?      -> _retire, or return
 `- plan_path
     |- Grid(map)
     |- planning_mask                   despeckle, dilate, free & not grown wall
     |- reachable                       flood fill from the root
     |- Grid(frontiers) -> cells        value above 50
     |- cluster_cells -> cluster_goal   up to 6 goals, nearest first
     |- plan_to -> rrt_star             for each goal. 1500, then 15000 iterations
     |- score                           H = L - w*I + c*turn
     |- best goal: set self.goal
     `- densify -> _make_path           points every 0.10 m, frame "map"
 `- path_pub.publish, _publish_markers
```

Then the follower takes over. It has no knowledge of the planner. It sees a new `Path` and drives.

---

## What is given and what is ours

| Part | Given | Ours |
|---|---|---|
| frontier detector | all of it | nothing |
| path follower | pop, P control, 0.3 rad rule | scan slow-down, stop at the end, TF guard, valid-return filter |
| launch file | all nodes, slam_toolbox | `use_sim_time` to every node, `task1` switch |
| SLAM config | all of it | four timing values |
| navigation node | TF helper, callbacks, skeleton | RRT*, mask, frontier goals, score, goal handling, markers, RViz goal |
| Task 1 script | no | `task1_baseline.py` |
| worlds, Gazebo launch | no | `lab3/sim/` |

Size: the navigation node has 614 lines in one file. The classmate repo has about 570 lines in one
navigation file.

---

## Reading order for the code

Do this after you answer the questions for T1 to T3, not before. For each step, say what the code
does before you read the comments.

1. `navigation_node.py`, `DEFAULTS` and the node's `__init__`: what does it subscribe to and publish?
2. `_tick` and `_retire`: when does it plan and when does it wait?
3. `plan_path`: the order of the steps. Match it to the call picture above.
4. `Grid` and `planning_mask`: map to mask.
5. `rrt_star`: sample, steer, parent, rewire, hit.
6. `cluster_cells`, `cluster_goal`, `score`: frontier to goal to H.
7. `path_follower_node.py`, `follow_path`: pop, speed, turn, scan factor.
8. `exploration.launch.py`: the node list and the arguments.

## Things to say about the code to the teacher, before he finds them

- The information term counts frontier cells, not unknown cells (T4).
- The planner runs on a 1 s timer, not in the map callback.
- The robot goes to the nearest free cell if it stands inside the collar.
- We cross off unreachable goals for good within 0.30 m.
- The scan frame is treated as `base_link`.
- The first Gazebo runs and the robot runs are listed in `LOG.md` with their labels.
