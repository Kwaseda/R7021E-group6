# Lab 3: why each choice, and what the teacher may ask

Read this after `THEORY.md`, `CODEMAP.md` and `ORAL.md`. It covers what changed on 2026-10-10
in Gazebo. Every number is [gazebo] unless it says Lab 1 bag. Run table:
`results/2026-10-10-overview.md`.

## What the lab asks for, and where it is

| Lab PDF | Ours | File |
|---|---|---|
| T1: send a Path, send a new one at the end | `task1_baseline`, 0.5 m forward and back | `task1_baseline.py` |
| T2: RRT/RRT* from scratch, on the live SLAM map, rooted at the robot, shortest branch to a known goal | RRT* with rewiring; goal clicked in RViz on `/goal_pose` | `navigation_node.py`: `rrt_star`, `plan_to` |
| T3 (optional): keep the point-robot RRT off the walls | map inflation, 0.105 m = 3 cells | `planning_mask` |
| T4: key points from frontiers, information gain heuristic, short LiDAR range for gain | 8-connected frontier clusters, H = L - w*I + c*turn, I counted within 0.75 m | `cluster_cells`, `score` |
| T5: loop until explored or a criterion is met | 1 s timer: arrived / stalled / path old, then plan again; stop after 10 empty cycles | `_tick` |
| Video and RViz recording | bag, then `video/bag_to_video.py`, or a screen recording | `RUNSHEET.md` |

The PDF says the focus is the planner and the exploration method. The path follower, SLAM and
the frontier detector are given. Our changes to the follower are fixes for real runs, not the
graded part. Say so if asked.

## The why behind each value

| Value | Why |
|---|---|
| `inflation_radius` 0.105 m | Burger radius. Rounded up to 3 cells, so a 0.15 m collar. Corridors under about 0.40 m vanish from the planner |
| `step_size` 0.30, `rewire_radius` 0.60 | Short enough for 0.7 m corridors. Rewire radius twice the step, so a new node sees its neighbours |
| `max_iterations` 1500, retry 15000 | Desk test: 1500 solved 6 of 25 long paths, 15000 solved all. The longest Gazebo plan took 3.8 s |
| `info_radius` 0.75 m | The PDF tip: a short LiDAR range for the gain makes the robot explore near, not stare far |
| `info_weight` 0.10 m per cell | One frontier cell is worth 10 cm of driving |
| `look_ahead` 0.12 m | 0.20 aimed past corners into walls (small-2). 0.12 is about one 0.10 m waypoint |
| `kp_yaw` 1.0, `max_w` 0.6 | Gazebo odometry reported 41 % more turn than the truth when turning fast while driving. The SLAM pose drifts between scan updates |
| loop closure off | Twice a false closure in a maze where every corridor looks alike: pose error 0.15 m to 1.7 m in 4 s |
| back-off 0.10 m after 2 s blocked | The follower froze at corners and goals were crossed off for good (small-1) |
| stalled goals retried 2 times | One stall at a corridor mouth ended the whole run |
| stop on Ctrl-C | `turtlebot3_node` keeps the last command. Ctrl-C left the robot at 0.15 m/s until this fix. rclpy's own signal handler closes the context before a stop can be sent, so the follower starts without it |

## Questions you should answer without notes

1. **Why RRT* and not plain RRT?** RRT finds a path. RRT* rewires so the path gets shorter with more samples. T2 asks for the shortest branch.
2. **Why is the robot a point to the planner, and how do you fix it?** Collision checks test cells along an edge. Inflating walls by the robot radius makes a point plan safe for a disc.
3. **Why frontiers in clusters?** Single cells are noise and too many goals. A cluster is one opening into unknown space. Its middle, or the nearest free cell, is the key point.
4. **Write H.** H = L - w*I + c*|turn|. L path length, I frontier cells within 0.75 m of the goal, turn the first heading change. Lowest H wins. w = 0 is pure nearest goal.
5. **Greedy vs complete?** A high w chases big frontiers far away (greedy gain). w = 0 goes to the nearest (complete but slow). The competition scores 90 % time plus 100 % time.
6. **When does it stop?** After the first goal, 10 plan cycles in a row with no reachable goal, after giving stalled goals 2 more tries.
7. **Why TF and not /odom for the robot pose?** SLAM corrects odometry drift through map -> odom. In Gazebo odometry drifted 3 m; SLAM stayed under 0.35 m.
8. **What fails in real time?** Corner freezes, wrong poses between SLAM updates, false loop closures, a door that closes behind the robot in the map, late Wi-Fi data. Each one is in `LOG.md` with the run that showed it. This is report section 4.
9. **Why does Ctrl-C need code?** The robot has no command timeout. Without a zero on exit it keeps driving.
10. **use_sim_time?** True in Gazebo, so all nodes use /clock. Left out on the robot.

## Hall rules (unknown maze)

- Measure the narrowest passage. Under about 0.40 m the planner will not enter it. 0.40 to
  0.50 m: the robot may get in and then the padding closes the door behind it, and the run ends
  early with green frontiers left. Restart the launch if so. 0.20 m: the Burger (0.178 m) cannot
  pass. Say so to the teacher.
- Set the robot clock first (`RUNSHEET.md`). The Lab 1 robot was 325 days behind.
- Keep the stop command ready. Bad Wi-Fi delayed data up to 3.6 s in Lab 1. Use `max_v` 0.10 then.

## Why the follower keeps its scan check

With the scan check off (`stop_distance` 0.0, `slow_distance` 0.01), the small maze was explored
fully in 2 of 2 runs. In the big maze the SLAM pose went 2.6 m wrong, and the robot hit a wall:
its centre came 0.038 m from a wall, and the Burger's half width is 0.089 m. Map inflation only
helps while the pose is right. The scan check uses the laser directly, so it still works when
the pose is wrong. That is why both stay. If the teacher asks why there are two layers, this run
is the answer: `results/2026-10-10-lab3_maze-noscan-1`.
