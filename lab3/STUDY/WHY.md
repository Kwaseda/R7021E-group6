# Lab 3: why, and what the teacher may ask

Read `CODE.md` first. It has every file, function and value with its reason.

## What the lab asks, and where it is

| Lab PDF | Ours |
|---|---|
| T1 send a Path, a new one at the end | `task1_baseline.py` |
| T2 RRT/RRT* from scratch, live SLAM map, root at robot, shortest branch to a known goal | `rrt_star()`; RViz "2D Goal Pose", cheapest node within 0.15 m of the click |
| T3 keep a point robot off walls | `free_mask()`: walls grown by 0.105 m. Plus the follower's laser stop |
| T4 next-best-view: info gain at edge nodes of the RRT; short laser range | leaves scored `H = L - 0.10*I + 0.15*turn`, I within 0.75 m |
| T5 loop until explored | 1 s timer; stop after 10 plans with no leaf seeing a frontier |

The PDF allows frontier or NBV methods. We tried frontier clusters first (`lab3-old`, 29 values,
4 of 4 small mazes full). We switched to NBV because it is what the PDF describes for RRT, it
needs no clustering or goal search, and the tree already proves each candidate is reachable.

## Questions to answer without notes

1. **RRT vs RRT\*?** RRT\* picks the cheapest parent nearby and rewires, so branches get shorter. T2 wants the shortest branch.
2. **Why inflate?** Edges are checked as lines of cells, so the robot is a point. Walls grown by the robot radius make a point plan safe for a disc.
3. **Why leaves?** The PDF: evaluate gain at edge nodes. Leaves are where the tree reaches the edge of known space.
4. **Why only free cells as samples?** No sample is wasted, and the tree never grows into unknown space.
5. **Why can the tree not reach unreachable places?** Every node joins by a free edge, so reachability comes for free.
6. **Why `seen_radius`?** Gain counts frontier within 0.75 m, even through walls. Some frontier cells never clear. Without the filter the robot crept 0.2 m at a time forever.
7. **Greedy vs complete?** Large `info_weight` chases big gain far away. 0 picks the nearest leaf that sees anything.
8. **Why TF, not /odom?** SLAM corrects drift with map -> odom. Gazebo odom drifted 3 m, SLAM stayed under 0.35 m.
9. **Why loop closure off?** Two false closures in a maze where all corridors look alike.
10. **Why does the follower stop on Ctrl-C in code?** The robot keeps its last command. Proved in Gazebo.
11. **Why a laser stop as well as inflation?** Inflation needs a correct pose. With the laser stop off the robot hit a wall when SLAM was 2.6 m off.
12. **use_sim_time?** True in Gazebo, left out on the robot.

## Hall rules

- Narrowest passage: under about 0.40 m it is never entered. 0.20 m: the Burger (0.178 m) cannot pass.
- Set the robot clock first (RUNSHEET). Keep the stop command ready. Bad Wi-Fi: `max_v` 0.10.
- Tested: 4 m maze full. Big maze result in `LOG.md`.
