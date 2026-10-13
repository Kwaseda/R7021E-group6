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

Commit 8b823b2 ("Lab 4 params") also contains Lab 3 files that were already staged: RUNSHEET.md, navigation_node.py, path_follower_node.py, bag_to_video.py, bag_to_report.py and a .gitignore rule. We keep the history as it is. Before each commit we now check the full staged list with git diff --cached --name-only.
