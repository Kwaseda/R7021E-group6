# Lab 3 log

Earlier entries (desk version, frontier clusters, all Gazebo debugging): `../lab3-old/LOG.md`.

## 2026-10-11: next-best-view planner

### What we did
- Copied `lab3-old`. Replaced only `navigation_node.py` (330 lines, 13 values, was 638 and 29) with NBV: one RRT* tree, leaves scored by frontier gain. Follower, launch, SLAM config, Task 1, sim worlds, video unchanged.
- Kept the fixes the 2026-10-10 runs proved: follower stop on Ctrl-C, laser stop and back-off, look_ahead 0.12, slower turns, loop closure off, `/inflated_map`.
- Took ideas from the other two groups (NBV, leaf scoring, replan on arrival) and fixed their weak points: planning on a timer, not in the map callback; 0.20 m arrival plus stall and age rules; a stop rule that holds the robot; turn measured on the first branch edge, not the bearing.

### What went wrong
- nbv-small-1: the robot crept 0.2 m at a time toward frontier cells it could not clear (gain counts through walls). Fix: `seen_radius`. Not committed separately, it is part of the first version.

### Results [gazebo]
- nbv-small-2: full, 15.5 m2, 90 % at 168 s, 0 stalls, 0 back-offs, closest 0.14 m, SLAM error 0.22 m, longest plan 0.26 s.

### Not tested
- The robot. The big maze (see below). RViz click on the NBV node. The narrow world.
