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
