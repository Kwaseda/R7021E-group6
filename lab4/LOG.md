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
