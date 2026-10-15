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
- nbv-big-1 [gazebo]: FAILED. 14.5 of about 52 m2, 74 stalls, 303 back-offs, 30 min limit. The planner picked the same unreachable leaves again: on a stall only the robot's spot was remembered. Fix: on a stall the goal is remembered too. That fix is NOT tested yet. Result of the next run: `results/2026-10-11-nbv-big-2/summary.md`.
- Until nbv-big-2 is full, the proven version for a maze larger than 4 m is `../lab3-old` (small 4 of 4 full, big 1 of 2 full).
- nbv-big-2 [gazebo], with the stall fix: cut at 1124 s by a laptop reboot (no launch log, bag not closed). Read from the raw mcap: 38.8 m2 known and still growing. Flat at 15.1 m2 from 192 s to about 700 s, then 27.0 m2 at 843 s. Better than nbv-big-1 (14.5 m2 after 30 min), worse than lab3-old (44 and 49 m2 at about 1200 to 1300 s). The 8.5 minute plateau is the open problem.
- Decision for the hall: maze about 4 m or smaller, use lab3. Larger, use lab3-old.

## 2026-10-12: circling at the end of a run

- Dominic's small maze run: after 637 s every plan had I = 1 to 4 and L = 0.2, and the robot
  hopped around one spot for two minutes at 12.5 m2. The frontier cells sat behind a wall,
  0.5 to 0.75 m from any spot the robot could reach, so `seen_radius` (0.5 m) never removed them
  and `info_radius` (0.75 m) kept counting them.
- Fix: `min_gain` 5. A leaf must see at least 5 frontier cells. Real goals in that log had I of
  10 or more.
- This is the value tested for the runs recorded in this historical entry. The current source
  default is 7; use `STUDY/CODE.md` and `navigation_node.py` for the active setting.
- After the fix [gazebo]: small maze full, 15.3 m2, `exploration finished` after 271 s.
  `lab3_maze_blind_a` (5.6 m) twice: both full and finished, 30.4 and 30.3 m2 of 31.4, after 456
  and 441 s, 0 stalls, 0 and 3 back-offs.
- RUNSHEET rewritten as a terminal-by-terminal procedure for the hall and for Gazebo.

## 2026-10-15: info_weight comparison in Gazebo

Done after the hall runs, as a check of the info_weight 0.10 used in the hall. Not done before
choosing it. World lab3_maze_blind_a (5.6 by 5.6 m, 31.4 m2, about the hall size). Everything
else fixed at the hall values: min_gain 8, inflation_radius 0.12. Two runs per value. All
numbers [gazebo]. Times are sim seconds after the first plan.

| run | finished | plans | stalls | back-offs | final m2 | 90 % at (s) | within 0.2 m2 of final at (s) |
|---|---|---|---|---|---|---|---|
| iw0.0-1 | yes | 173 | 2 | 2 | 29.9 | 394 | 493 |
| iw0.0-2 | yes | 196 | 1 | 6 | 30.9 | 464 | 515 |
| iw0.10-1 | yes | 49 | 0 | 0 | 30.4 | 389 | 437 |
| iw0.10-2 | yes | 49 | 0 | 0 | 30.8 | 298 | 403 |
| iw0.30-1 | yes | 45 | 1 | 8 | 30.2 | 301 | 451 |
| iw0.30-2 | yes | 37 | 1 | 2 | 29.2 | 364 | 450 |

Mean of the two runs (90 % time + time to final, the PDF competition score):

| info_weight | score | plans | stalls | back-offs |
|---|---|---|---|---|
| 0.0 (nearest leaf that sees 8 cells) | 933 s | 185 | 1.5 | 4 |
| 0.10 (hall value) | 764 s | 49 | 0 | 0 |
| 0.30 (greedier) | 783 s | 41 | 1 | 5 |

- All six runs explored the maze fully (29.2 to 30.9 m2 of 31.4) and ended with
  exploration finished.
- 0.0 is the slowest in both runs and makes about 4 times more plans. With no reward for gain it
  takes many short hops to the nearest leaf.
- 0.10 and 0.30 cannot be separated with two runs: the spread inside one value (298 to 389 s for
  0.10) is larger than the difference between them. 0.10 was the only value with no stalls and no
  back-offs in both runs.
- Conclusion for the report: some weight on gain clearly helps; the hall value 0.10 is as good as
  0.30 here and drove more cleanly.
