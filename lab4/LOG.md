# Lab 4 log

Dated entries. Every number has a label: [desk], [gazebo] or [hall].

## 2026-10-13: desk check of the five modules

Scripts are in `lab4/desk/` (not in git). One script for each module. Each script checks the output shape and one case that we calculated by hand.

| Module | Check | Result |
|---|---|---|
| motion_model | Turn pi/2, drive 1 m from origin with zero noise gives (0, 1, pi/2) | pass [desk] |
| motion_model | Spread of 20 000 samples with sd_trans 0.1 is 0.1 | pass [desk] |
| motion_model | Log density at the exact pose is -log(s1 s2 s3) - 1.5 log(2 pi). It agrees with `utils.odometry_increment` | pass [desk] |
| resampling | N_eff is 4, 1 and 2 for uniform, one-hot and two-half weights | pass [desk] |
| resampling | Weights [0.5, 0.25, 0.25, 0] give counts [2, 1, 1, 0] in 100 draws | pass [desk] |
| measurement_model | Wall 1 m ahead. Log-likelihood at 0, 0.1 and 0.3 m offset is -4.8, -8.0, -19.0. Off-map pose gives every beam at max_dist | pass [desk] |
| proposal | K = 27 gives a 3x3x3 lattice with offsets +-w/2. Sigma is symmetric and positive definite. Mean is (0, 0, 0) at the true pose | pass [desk] |
| rbpf.step | 5 particles, 4 m square room, 20 steps of 0.05 m. True end x = 0.95 m. FS1 best x = 0.961, FS2 best x = 0.994. N_eff stays in [1, 5]. Weights sum to 1 | pass [desk] |

Two problems came from the check, not from the modules:

1. `Config.range_max` is 0 by default. The node reads it from the first LaserScan. The desk check sets 3.5 m.
2. The shipped `config/params.yaml` is a template too. `p_free`, `p_occ`, `hit_window_cells`, `log_odds_limit`, `odometry_sigmas`, both scan match windows, `match_score_min`, both proposal windows and `resample_threshold` are 0. `map_resolution` is 0.01 m. With these values the node cannot run the filter: the scan matcher divides by a window of 0, and `p_occ` 0 gives a log-odds of minus infinity. We must fill in these values before step A.

The desk check used its own values for these fields (in `desk/_path.py`). They are not tuned and they are not in `params.yaml`.

The rbpf result is one seed on a synthetic room. It shows that the step runs and the numbers are sensible. It does not show which filter is better.

## 2026-10-13: first values for params.yaml

The shipped yaml had 0 in 14 fields. We chose a start value for each. All are starting points for Task 1 and Task 7 tuning. None is tuned yet.

| Field | Old | New | Why |
|---|---|---|---|
| map_resolution | 0.01 | 0.05 | 0.01 m on 50 m is 25 M cells, about 100 MB for each particle. Too large for the laptop and not necessary for the maze. 5 cm is larger than the LDS range noise |
| map_size_m | 50 | 20 | 400 x 400 cells, about 0.6 MB for each particle. The maze fits |
| p_occ | 0 | 0.7 | +0.85 log-odds for each hit. One stray return does not make a wall |
| p_free | 0 | 0.4 | -0.41 for each pass. Weaker than a hit, so one grazing beam cannot erase a wall |
| hit_window_cells | 0 | 2 | +-5 cm around the hit, the size of the range noise |
| log_odds_limit | 0 | 5 | Caps p at 0.993, so a wrong cell can recover after 6 to 12 scans |
| odom_noise_sigma | 0 | 0.003 each | Sim only. Added on each /odom message, not on each filter step. Set [0, 0, 0] for the hall |
| odometry_sigmas | 0 | 0.02 each | A few times the noise that adds up in one filter step. Too narrow and the particles never cover the true pose |
| scan_match_window_xy | 0 | 0.2 | Half-width, +-20 cm. Final step w/4 = 5 cm = one cell |
| scan_match_window_theta | 0 | 0.1 | +-0.1 rad, about 3.5 sigma of the heading prior. Final step 0.025 rad |
| match_score_min | 0 | 0.3 | The first steps have an empty map and score 0, so they fall back. That is correct |
| num_candidates | 3 | 27 | 3 rounds down to 1 candidate, and then Sigma is only the regularization floor. 27 is 3 x 3 x 3. 125 costs 4.6 times more |
| proposal_window_xy | 0 | 0.1 | Full width, +-5 cm. Covers the +-2.5 cm that the matcher leaves |
| proposal_window_theta | 0 | 0.05 | +-0.025 rad, the final matcher step |
| resample_threshold | 0 | 0.5 | Resample only when N_eff < N/2 (selective resampling) |
| resample_every_step | true | false | Resampling on every step removes diversity also when the weights are even. The Neff figure tests this |

The desk checks pass again with these values and no desk overrides [desk]. FS2 best pose after 20 steps: x 0.974 m, true 0.95 m [desk].

## 2026-10-13: mixed commit

Commit 8b823b2 ("Lab 4 params") also contains Lab 3 files that were already staged: RUNSHEET.md, navigation_node.py, path_follower_node.py, bag_to_video.py, bag_to_report.py and a .gitignore rule. Commit 10f835f also contains seven lab3/ study files (PLAN.md and STUDY/*.md). Another session stages Lab 3 files while we work. We keep the history as it is. From now on we commit with git commit --only <path>. That commits only the named paths, whatever else is in the index.

## 2026-10-14: smoke test and the first bug

Smoke run `runs/smoke` (N = 5, FS2, 87 s, 198 steps, a small loop of about 2.5 m) [gazebo]:

- No exception. The node wrote the .npz and the .json.
- Estimate error mean 0.29 m, max 0.86 m. Raw odometry error mean 0.04 m. The filter made the pose worse.
- The map has double walls, turned against each other.
- N_eff mean 1.8 of 5. It resampled on 161 of 198 steps.
- Fallbacks: 0 on every early step, but the map was empty then.
- Step time median 216 ms, p95 256 ms. That is more than the 200 ms scan period at N = 5.

Cause: the scan-match score is the mean occupancy probability at the beam endpoints (`scan_matcher.correlation`). An unknown cell has p = 0.5, so an empty map scores 0.5, not 0. With `match_score_min` 0.3 the fallback can never occur. The first steps trust a match against a map that holds one or two scans, and that wrong pose is then built into the map. Our first value of 0.3 came from the wrong assumption that an empty map scores 0.

Desk test of only this value, N = 5, a loop of radius 0.25 m in a room with inner walls, 3 seeds [desk]:

| match_score_min | Mean error per seed (m) | Fallbacks |
|---|---|---|
| 0.3 | 0.176, 0.152, 0.106 | 0, 0, 0 |
| 0.55 | 0.052, 0.057, 0.083 | 5, 5, 5 |
| 0.6 | 0.066, 0.057, 0.085 | 6, 5, 6 |

Fix: `match_score_min` 0.55, just above the score of an unknown map. The step time is a separate problem. We look at it after the Gazebo rerun.

## 2026-10-14: smoke test passes

Three runs, all N = 5, FS2, `lab3_maze_small` [gazebo]:

| Run | match_score_min | Time, path | Est error mean / max (m) | Odom error mean / max (m) | Fallbacks in first 20 steps | Step time median / p95 (ms) |
|---|---|---|---|---|---|---|
| smoke (Dominic, teleop) | 0.3 | 87 s, 2.5 m | 0.289 / 0.857 | 0.039 / 0.082 | 0 | 216 / 256 |
| smoke_b (Dominic, teleop) | 0.3 | 492 s, 12.8 m | 2.152 / 2.956 | 2.347 / 2.771 | 0 | 239 / 283 |
| smoke2 (loop, 0.05 m/s, 0.2 rad/s) | 0.55 | 146 s, 3.5 m | 0.215 / 0.401 | 0.166 / 0.310 | 5 | 234 / 280 |

Done condition of step A: no exception for more than 2 minutes, a particle cloud and a map like the maze. smoke2 meets it. Its map has single, sharp walls. The smoke map had double walls.

Things we saw, for later steps:

1. In smoke2 the yaw error of the estimate moves to about -0.3 rad in the first 75 s, while the map is small. After that it stays constant, while the odometry yaw error continues to change. The map is then correct in itself but turned by about 0.3 rad. `evaluation.absolute_trajectory_error` does not align the trajectories, so this early drift counts in the ATE. This is Task 7 tuning.
2. The node adds `odom_noise_sigma` on each /odom message, also when the robot stands still. /odom is 44 Hz [gazebo]. With 0.003 on rot1 and rot2, the yaw noise grows by about 0.028 rad for each square root of a second, also without motion. In smoke_b the robot stood for 124 s and the noisy odometry turned by -0.575 rad. Before we record the bag (step B4), decide if 0.003 is the correct value at 44 Hz.
3. /scan is 4.4 Hz in Gazebo, so the scan period is 225 ms. The step time at N = 5 is already 234 ms median. Live, the node drops scans. For the sweep, replays at N = 20 and N = 50 must be slowed down or use a deep `scan_queue_depth` (step C9).
4. Before smoke2, an orphan `gz sim -r -s` server was still running after the launch had stopped. No /odom or /scan was visible. `pkill -f "gz sim"` fixed it.

## 2026-10-14: the one read of the classmates' code

Source: github.com/willmerw/R7021E-, lab4/src/r7021e_fast_slam, cloned to ~/reference/willmerw (outside our repo). We read it one time, after our smoke test. We copied no code and no values. Do not read it again. Everything we learned is here.

What is the same as ours:

- Helper files: `config.py` and `scan_matcher.py` differ from the template, but only by one space and one pair of parentheses. All other helpers and the node are identical. The handover said "identical", which is true in effect.
- The five modules are the same size as ours (580 lines against 606) and use the same method: the same motion model, the same likelihood field, the same systematic resampling, and the same Gaussian fit in log space. There is nothing that would make our code smaller.
- Their `match_score_min` is 0.7 in the yaml and 0.85 in the sweep. Both are above 0.5. This agrees with our bug of today.

Differences that are not errors:

- Their proposal lattice spans +-w. Ours spans +-w/2, as our docstring says. Both are valid definitions. At the oral, say which convention we use and why the scan matcher is different (+-w).

Defects in their work. Do not repeat them:

1. Their sweep is not comparable across N. Their runs have 36 to 37 filter steps at N = 1 and only 9 to 12 at N = 100. With `scan_queue_depth` 1 and replay in real time, the node drops the scans it cannot process in time. A run with large N then sees a different and shorter set of scans. Our rule for step C9: before we compare ATE, all runs of the sweep must have the same number of steps.
2. Their sweep used `odom_noise_sigma` 0.001. Their ATE is about 0.03 to 0.07 m for all N and for FS1 and FS2. The experiment was too easy to show a difference. This is a reason to keep noise that is not trivial (our 0.003, see the smoke test finding 2).
3. In their `step()`, `np.argmax((... for p in ...))` gets a generator, and numpy then returns 0 every time. Their logged `best_index` is always 0. The node does not use that column for the estimate, so their ATE is not affected.
4. Their `step()` does not time the improved proposal branch. Only the fallback branch goes into `t_likelihood`. Their timing figure shows FS2 as faster than it is. For our comparison run (step C13), the logged step times of the two packages are therefore not comparable. Use the wall time of the replay, or compare only `t_scan_match` and `t_map_integrate`.
5. The yaml in their repo does not reproduce their sweep. The sweep .json files show `map_resolution` 0.01 and scan match and proposal windows of 3.0 m. The committed yaml has other values. For our report, the .json that the node writes with each run is the record of the values.

What we adopt: no code and no values. We adopt one process rule (item 1: equal step counts in the sweep) and one argument (item 2: the noise must not be trivial). Neither changes our code.

## 2026-10-15: the drive bag runs/maze_drive

Claude drove the robot with a desk script (`desk/drive_maze.py`, not in git), at Dominic's request. Route in `lab3_maze_small`, on the corridor centre lines: (0,0) to (-1.6,0), (-1.6,-1.6), (0,-1.6), (0,-0.8), (1.6,-0.8), and back the same way to (0,0). The maze is a tree with no loop, so the robot returns along the same corridors. 0.15 m/s, turns at 0.3 rad/s.

Result [gazebo]: 218 s, 13.9 m, 816 scans. The closest wall over the whole drive was 0.29 m, with no contact. The true end pose is 3.4 cm from the true start pose. Topics: /scan /odom /tf /tf_static /clock /ground_truth.

Earlier attempts, kept in `desk/` and not used:

- `runs/maze_drive_dominic` (teleop, 175 s): the end is 0.86 m from the start, and 346 of 771 scans have a wall closer than 0.15 m. The robot body touched walls.
- Two driver attempts that steered on /odom. In one the robot stuck against a wall. In the other it stopped 0.13 m from a wall end.

### Finding: Gazebo /odom is not ground truth

The node logs /odom as ground truth in simulation. But /odom in Gazebo is wheel odometry. On this drive, without any contact, /odom was 0.171 m from the true pose on average, 0.312 m at most, and 0.057 rad in yaw on average [gazebo]. The true pose comes from Gazebo itself (`/world/maze_world/dynamic_pose/info`, entry 0 is the model `burger`). A bridge and the driver republish it as `/ground_truth` (nav_msgs/Odometry, stamped with sim time).

Effect: the smoke runs of 2026-10-14 used /odom as ground truth. Their error numbers include up to about 0.3 m of Gazebo odometry drift. They are not comparable with later runs.

Rule from now on: every replay of `runs/maze_drive` uses `ground_truth_topic:=/ground_truth`. The node supports this parameter already (grid_slam_node.py, `ROS_DEFAULTS`). We did not change the node.

### Real robot odometry, rough only

The Lab 3 hall bags (`lab3/ros2_ws/bags`) have /odom and /tf, and no /clock, so they come from the real robot. Two rough facts [hall, Lab 3 bags]:

1. The real /odom rate is about 19 Hz. Gazebo is 44 Hz. The node adds `odom_noise_sigma` on each message, so the same value gives less noise per second on the real robot. In the hall, `odom_noise_sigma` is [0, 0, 0] in any case.
2. slam_toolbox corrected the odometry by 0.06 m and 0.02 rad over 8.8 m and 50 rad of turning (task4 bag). The heading of the real robot drifts very little, probably because turtlebot3_node uses the IMU for the heading.

Limits: these runs were Lab 3 exploration, with a different SLAM (slam_toolbox), other speeds and other driving. A slam_toolbox correction is not a measurement of truth. We do not tune Lab 4 values from these numbers. The hall tuning (Task 8) happens in the hall.

## 2026-10-15: params.yaml for sim replays

Two values in `params.yaml` are for the simulation only. The course's `tools/run_sweep.sh` reads `params.yaml` directly, and the launch file only accepts num_particles, seed, use_improved_proposal and run_name as overrides. So these values must be in the file:

- `scan_queue_depth: 1000`. In a replay the node must process every scan, also when a step is slower than the scan period. Otherwise a run with a large N sees fewer scans, and the ATE is not comparable (classmates' defect 1).
- `ground_truth_topic: /ground_truth`. The true Gazebo pose in `runs/maze_drive`.

Before the hall: `scan_queue_depth: 1` and `ground_truth_topic: ''`, together with `odom_noise_sigma: [0, 0, 0]` and `use_sim_time` false.

## 2026-10-15: Task 1, occupancy mapping with known poses

Both runs replay `runs/maze_drive` at real-time rate with `use_measurement_update: false`, N = 1, `odom_noise_sigma` 0.003, deep scan queue and `/ground_truth`. `mapping_shipped` uses the occupancy values of the first commit: `map_size_m` 50, `map_resolution` 0.01, `p_free` 0, `p_occ` 0, `hit_window_cells` 0, `log_odds_limit` 0, `use_clamping` true. `mapping_tuned` uses the values in `params.yaml`. Figure: `figures/maps.png`.

### Task 1a: what we changed, why, and the effect

We changed one value at a time, and each step kept the previous changes. These steps were replayed at 4 times real time into `desk/runs` [gazebo].

| Step | Change | Occupied | Free | Effect |
|---|---|---|---|---|
| shipped | none | 0 m2 | 0 m2 | 100 % unknown. With `log_odds_limit` 0 and clamping on, each update is clamped back to 0 |
| 1 | `log_odds_limit` 0 to 5 | 0 m2 | 7.9 m2 | The cells can now change. But `p_occ` 0 gives logit(0), a large negative value, so a hit adds free evidence. No walls |
| 2 | `p_occ` 0 to 0.7 | 0 m2 | 13.3 m2 | Still no walls. `hit_window_cells` 0 gives an occupied band of zero width |
| 3 | `p_free` 0 to 0.4 | 0 m2 | 11.7 m2 | Free evidence is now weaker (-0.41 for each pass instead of a very large value). No walls yet |
| 4 | `hit_window_cells` 0 to 2 | 0.63 m2 | 13.4 m2 | A few wall cells. At 1 cm the endpoints with noise seldom hit the same cell two times, and the free evidence of nearby beams removes them |
| 5 | `map_resolution` 0.01 to 0.05, `map_size_m` 50 to 20 | 5.16 m2 | 9.8 m2 | The walls are visible. The resolution was the important value. This is the tuned set |
| 6 | as 5, `use_clamping` false | 5.21 m2 | 9.95 m2 | Almost no difference. On a drive of 3.5 minutes, few cells get to +-5 |

The tuned map has double walls that are turned against each other. This is correct for Task 1: the map uses the raw corrupted odometry. The pose error against the true pose is 0.47 m mean and 0.98 m max [gazebo]. Task 7 must correct this.

### Limitation: the two runs did not use the same noisy trajectory

The node adds odometry noise on each /odom message that it receives. The odometry queue is 100 messages and the node has one thread. When a scan takes a long time (the 1 cm map), messages are dropped, and there are fewer noise draws. So the same bag and the same seed give a different noisy trajectory. Pose error against the true pose: `mapping_shipped` 1.64 m mean, 3.26 m max (149 steps). `mapping_tuned` 0.47 m mean, 0.98 m max (564 steps) [gazebo]. Two runs with the same seed and almost the same load (t1_5, t1_6) were already different at the first step.

Decision (Dominic): keep the noise in the node, as the course does, and state this limitation. We read the run logs of the classmates to check the course level: their `mapping_shipped` had 4 steps and noise 0.001, and their `mapping_tuned` had 151 steps and noise 0.005. Their two maps were not made from the same trajectory either.

Note: the bag has almost no standing time at the start. The robot moves 4.4 s after `/ground_truth` starts. The sim time of about 51 s at the first step is the Gazebo clock, not a wait.

## 2026-10-15: Task 7 tuning, baseline

All tuning runs: `runs/maze_drive`, N = 10, seed 1, replay at 0.5 times real time (the node then keeps up better, so it drops fewer /odom messages), logs in `desk/runs`. Summary script: `desk/t7_summary.py`, which uses the course's `evaluation.absolute_trajectory_error`.

Baseline with the first values in `params.yaml` [gazebo]:

| Run | ATE rmse / max (m) | Raw odometry rmse (m) | Yaw error at end (rad) | N_eff mean, resamples | Step time median (ms) |
|---|---|---|---|---|---|
| FS2 | 0.472 / 0.774 | 0.581 | -0.411 | 4.0, 441 of 570 | 233 |
| FS1 | 0.112 / 0.239 | 0.348 | -0.045 | 4.2, 413 of 578 | 106 |

The raw odometry of the two runs is different (0.58 m against 0.35 m), because of the noise limitation in the Task 1 entry. So this comparison is not paired.

What FS2 does with the heading: in the first 23 s the yaw error of the estimate follows the odometry to -0.24 rad. After that the filter holds it near -0.2 to -0.3 rad while the odometry goes to -0.97 rad. So the filter corrects drift after the map exists, but the early error is in the map.

Our explanation: the scan matcher adds a prior toward the odometry prediction. Its heading sigma comes from `odometry_sigmas` (hypot(0.02, 0.02) = 0.028 rad). The match score is a mean probability in [0, 1], so a heading correction of 0.025 rad costs about as much prior penalty as it gains in score. The match then stays at the odometry heading, and FS2 builds all particles around that match. FS1 instead spreads each particle by 0.02 rad for each step, and the measurement selects the best one.

Next test, one change: `odometry_sigmas` [0.05, 0.02, 0.05] (heading terms 0.02 to 0.05). This makes the heading prior weaker. The hall maze has more turns than the Lab 3 maze, so the heading terms are important there too.

## 2026-10-15: Task 7, the noise makes single runs unreadable, and the split

Test `odometry_sigmas` [0.05, 0.02, 0.05], FS2, N = 10, seed 1 [gazebo]: ATE rmse 0.360 m, raw odometry rmse 0.300 m, yaw error at end -0.390 rad. Baseline FS2: ATE 0.472 m, raw odometry 0.581 m. We cannot read this result. The raw odometry changed from 0.58 m to 0.30 m with the same bag and the same seed.

Cause: the node uses one random generator for the odometry noise and for the filter. When a filter value changes, the filter uses a different count of random numbers (fallbacks, samples). After that, the odometry noise gets different numbers. So each change of a parameter also changes the noise. The dropped /odom messages (Task 1 entry) add to this. One run per setting cannot show if a change helped.

What the sources say:

- Classmates: noise in the node (0.001 in the sweep, 0.005 for `mapping_tuned`), seeds 1 to 3, no control of this effect. They did not tune one value at a time.
- Lecture 9: no guidance about simulation noise, seeds or evaluation. It does support our diagnosis of the FS2 heading. For a laser, p(z | x, m) "is typically peaked and dominates the product". The measurement limits the area locally, and the odometry limits it only globally. In our baseline the odometry prior controlled the heading.
- Lectures 8 and 10: nothing about this.
- Lab PDF section 4.1: "Simulated odometry is too good", so the node corrupts it, and the clean pose is the ground truth. On our drive this assumption is not true: Gazebo /odom is 0.17 m from the true pose. The PDF asks for the sweep with at least three seeds, "so the plot includes a spread".

Decision (Dominic), a split:

1. Tuning (our process, not a deliverable): `odom_noise_sigma` [0, 0, 0]. The odometry error is then the error of Gazebo's own wheel odometry against `/ground_truth`. It is in the bag, so it is the same in every replay, and one run for each setting is sufficient.
2. Deliverables (sweep, Neff figure, maps figure): `odom_noise_sigma` 0.003, as PDF section 4.1 asks. The seeds give the spread. The change of the noise between runs is then part of that spread, and we state it as a limitation.
3. Risk: values tuned on a smaller odometry error can be too narrow for the sweep at 0.003. So we keep the heading sigmas wide, and we do one FS2 check at 0.003 before the sweep.

## 2026-10-15: Task 7, why FS2 loses its heading

FS2, N = 10, seed 1, `odom_noise_sigma` 0 (Gazebo odometry only) [gazebo]: ATE rmse 0.309 m, raw odometry rmse 0.193 m, yaw error at end -0.336 rad. FS2 makes the pose worse than the raw odometry.

Heading error that FS2 added, split by the motion of the robot [gazebo]:

| Motion | True turn (rad) | Added by FS2 (rad) | Added by odometry (rad) |
|---|---|---|---|
| Turn left | +7.04 | +0.178 | +0.067 |
| Turn right | -13.39 | -0.319 | -0.225 |
| Straight | +0.05 | -0.220 | +0.021 |

Hypothesis 1: ties in the scan matcher (argmax takes the first of equal scores, and the lattice starts at the most negative offset). Desk test, 200 matches from noisy start poses on a map built at true poses [desk]: mean heading error -0.0015 rad, with 50 % of the errors negative and 50 % positive. Not confirmed.

Finding from the same test: the matcher removes almost none of the start error. Start error 0.020 m and 0.020 rad (standard deviation). After the match: 0.020 m and 0.016 rad [desk]. The finest step of the matcher is 5 cm and 0.025 rad, larger than the error, and the prior pulls toward the start. Other values [desk]: windows 0.1 m / 0.05 rad gives 0.015 rad; `odometry_sigmas` 0.05 (weaker prior) gives 0.012 rad. The matcher is coarse by design (it reads cell centres).

Hypothesis 2: each FS2 step leaves a heading error of about 0.01 to 0.02 rad, and the errors add as a random walk. The node does a step every 0.05 m or 0.0175 rad (1 degree): 559 steps on 13.9 m. 0.01 x sqrt(559) = 0.24 rad. That is the size of the drift we see. gmapping (the implementation by Grisetti, the paper in Lecture 9) updates every 1 m or 0.5 rad by default. Test, one change: `update_min_dist` 0.2 and `update_min_angle` 0.2.

A run was spoiled: two replays ran at the same time for about one second (both publish /clock and /scan on domain 34). We deleted the FS1 run of that time and ran it again alone. `desk/replay.sh` now refuses to start when another replay runs.

Result of the test, FS2, N = 10, seed 1, `odom_noise_sigma` 0 [gazebo]:

| Run | Steps | ATE rmse / max (m) | Yaw error at end (rad) |
|---|---|---|---|
| Raw Gazebo odometry | | 0.192 / 0.315 | |
| FS2, steps every 0.05 m or 0.0175 rad | 559 | 0.309 / 0.541 | -0.336 |
| FS1, steps every 0.05 m or 0.0175 rad | 559 | 0.147 / 0.284 | -0.091 |
| FS2, steps every 0.2 m or 0.2 rad | 136 | 0.085 / 0.186 | -0.058 |

The raw odometry error is now the same in all runs (0.192 to 0.193 m), so this comparison is paired. Hypothesis 2 is confirmed. With fewer steps, fewer small heading errors add up. FS2 is now better than the odometry and better than FS1 with the old thresholds.

Change in `params.yaml`: `update_min_dist` 0.05 to 0.2, `update_min_angle` 0.0175 to 0.2. These two values are not in the Task 7 list of the PDF, but they are in `params.yaml`. They also make each run 4 times cheaper, which helps the sweep and the hall laptop at large N.

## 2026-10-15: Task 7, FS1 check and the check at 0.003

With steps every 0.2 m or 0.2 rad, N = 10, seed 1 [gazebo]:

| Run | Noise | ATE rmse / max (m) | Raw odometry rmse (m) | Yaw error at end (rad) |
|---|---|---|---|---|
| FS2 | Gazebo odometry | 0.085 / 0.186 | 0.192 | -0.058 |
| FS1 | Gazebo odometry | 0.103 / 0.226 | 0.192 | -0.075 |
| FS2 | 0.003 | 0.452 / 0.751 | 0.343 | -0.209 |

Paired, FS2 is better than FS1, and both are better than the odometry.

At 0.003 FS2 is worse than the odometry. Cause: the node adds the noise on each /odom message. One step of 0.2 m is now about 60 messages at 44 Hz. The noise in one step is then about 0.0042 x sqrt(60) = 0.033 rad in heading and 0.003 x sqrt(60) = 0.023 m in translation. `odometry_sigmas` 0.02 is narrower than this. With larger steps, the motion model must be wider.

Next test, one change: `odometry_sigmas` [0.05, 0.04, 0.05], at 0.003 and at Gazebo odometry.

Result, FS2, N = 10, seed 1, `odometry_sigmas` [0.05, 0.04, 0.05] [gazebo]:

| Noise | Before: ATE rmse / max (m) | After: ATE rmse / max (m) | Raw odometry rmse (m), before / after |
|---|---|---|---|
| 0.003 | 0.452 / 0.751 | 0.196 / 0.338 | 0.343 / 0.579 |
| Gazebo odometry | 0.085 / 0.186 | 0.105 / 0.162 | 0.192 / 0.192 |

At 0.003 the run pair is not paired (the raw odometry is different). But the ratio ATE / raw odometry goes from 1.32 to 0.34, which is much larger than that effect. At Gazebo odometry (paired) the mean gets a little worse and the maximum gets better. We keep the wider values: they are necessary at 0.003, and they are the safe side for the turns in the hall.

Change in `params.yaml`: `odometry_sigmas` [0.02, 0.02, 0.02] to [0.05, 0.04, 0.05].

## 2026-10-16: Task 7 sweep and the four figures

Sweep: `tools/run_sweep.sh runs/maze_drive "1 5 10 20 50" "1 2 3"`, 30 replays at real time, `params.yaml` as committed (noise 0.003, steps every 0.2 m or 0.2 rad, `odometry_sigmas` [0.05, 0.04, 0.05], deep scan queue, `/ground_truth`). No errors. Every run has 127 to 139 steps, so all runs used almost the same scans [gazebo].

ATE rmse over 3 seeds, mean (min to max), m [gazebo]:

| N | FastSLAM 1.0 | Grid-FastSLAM 2.0 |
|---|---|---|
| 1 | 1.081 (0.636 to 1.683) | 0.387 (0.272 to 0.476) |
| 5 | 0.342 (0.184 to 0.545) | 0.239 (0.199 to 0.279) |
| 10 | 0.210 (0.137 to 0.283) | 0.130 (0.088 to 0.197) |
| 20 | 0.175 (0.082 to 0.324) | 0.161 (0.070 to 0.326) |
| 50 | 0.101 (0.085 to 0.131) | 0.142 (0.121 to 0.180) |

Raw odometry rmse in these runs: 0.23 to 1.15 m. It changes between runs (shared random generator, Task 1 limitation).

Extra runs for figures 1 and 3 (N = 10 for N_eff, N = 20 for the map, seed 1, same values):

| Run | ATE rmse (m) | Resamples | N_eff / N median |
|---|---|---|---|
| `neff_selective` (`resample_every_step` false) | 0.197 | 104 of 135 steps (78 %) | 0.34 |
| `neff_every` (true) | 0.121 | 136 of 136 (100 %) | 0.40 |
| `fastSLAM2` (FS2, N = 20) | 0.070 | 106 of 135 | |

Figures (all made with the PDF commands; the two maps figures got their own names because both commands write `maps.png`):

### Figure 1, `figures/maps_task7.png` (and `maps_task1.png` for Task 1)

The map from raw corrupted odometry has double walls, turned by about 0.3 rad, and the path leaves the corridors. The FS2 map (N = 20) has single walls at right angles, and the estimate is on the true path (ATE 0.070 m). The filter corrects the heading drift that the raw odometry builds into the map.

### Figure 2, `figures/ate_vs_particles.png`

At N = 1 FS2 is almost 3 times better than FS1 (0.39 m against 1.08 m), and its band is much narrower. With one particle FS1 is dead reckoning with extra noise, while FS2 still corrects each step with the scan match. Both curves drop as N grows. From N = 10 the bands overlap, and at N = 50 both are near 0.1 m. FS2 needs fewer particles for the same error, as Lecture 9 says. The bands are wide because each run sees a different noise sample (one generator for the noise and the filter), so a difference smaller than the bands is not a result.

### Figure 3, `figures/neff.png`

N_eff often falls below N/2 even with selective resampling: 78 % of the steps resampled, against 100 % with resampling on every step. The median N_eff / N is 0.34. The scan likelihood is very sharp (about 300 beams, each counted as independent), so after one step most weight is on a few particles. Selective resampling saves only about one resample in five here. With a flatter likelihood (`sigma_hit` larger, or `likelihood_stride` 2 to 4) the difference would be larger. The ATE of the two runs (0.197 and 0.121 m) is one seed each with different noise, so it does not show which policy is better.

### Figure 4, `figures/timing.png`

Time per step grows linearly with N: about 35 ms at N = 1, 230 ms at N = 10, 830 ms at N = 50 for FS2. Each particle runs its own scan match (blue) and its own proposal with 27 likelihoods (orange); the map update (green) is small. Memory grows linearly too: one 400 x 400 grid per particle, 0.64 MB each, 32 MB at N = 50. A step only has to finish before the next gated scan, not before the next scan. With steps every 0.2 m or 0.2 rad, that is about 1.3 s on a straight and 0.4 to 0.7 s in a turn at our speeds. N = 20 (about 360 ms) fits. N = 50 (830 ms) falls behind in turns. On the robot (queue 1) the node then drops scans; in the replay the deep queue keeps them.

### What this means for the hall

Start the hall at N = 10 (ATE 0.13 m mean in simulation, about 230 ms per step). N = 20 also fits the step time. Do not go to 50.
