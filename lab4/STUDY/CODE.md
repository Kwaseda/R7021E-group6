# Lab 4 code, file by file

The package is the course template `r7021e_fast_slam`. We wrote five student files and the
values in `config/params.yaml`. Everything else is given. Find a function by its name, not by a
line number.

## How the parts meet

```
Gazebo or TurtleBot3 --/scan, /odom--> grid_slam_node.py (given)
                                          |  gate: 0.2 m or 0.2 rad since last step
                                          |  u = odometry_increment(prev, now)
                                          v
                                 rbpf.GridFastSLAM.step(u, endpoints, ranges)   <- OURS
                                          |  per particle:
                                          |   motion_model.sample_motion_model_odometry   <- OURS
                                          |   scan_matcher.match (given)
                                          |   proposal.improved_proposal                  <- OURS
                                          |     measurement_model.measurement_log_likelihood  <- OURS
                                          |     motion_model.motion_model_log_pdf         <- OURS
                                          |   grid_map.integrate_scan (given), LAST
                                          |  resampling.effective_sample_size             <- OURS
                                          |  resampling.systematic_resample               <- OURS
                                          v
                     /map, /particles, /slam_path, map->odom TF, runs/<run_name>.npz
```

---

## `motion_model.py` (Task 2)

**`sample_motion_model_odometry(u, poses, sigmas, rng)`**. One noisy copy of the increment
`u = [rot1, trans, rot2]` for each particle, then turn, drive, turn. Returns (N, 3).
Used in step 3: FS1 uses it as the whole proposal. FS2 uses it as the start point `x̄` of the
scan match.

**`motion_model_log_pdf(x_cand, x_prev, u, sigmas)`**. The density of the same model, for K
candidate poses that came from somewhere else (the proposal lattice). It splits each candidate
into the increment it implies, in the same way as `utils.odometry_increment`, compares with
`u`, and adds three Gaussian log densities. Angle residuals are wrapped.

Why both forms must agree: the proposal multiplies the laser likelihood by this density. If the
density described a different noise than the sampler, the weights would be wrong.

`_SIGMA_FLOOR = 1e-4`: no division by zero when a sigma is 0.

## `measurement_model.py` (Task 3)

**`measurement_log_likelihood(poses, endpoints, grid_map, cfg)`**. Likelihood field model. For K
poses: put all beam endpoints in the map frame (`transform_points`, shape (K, B, 2)), look up the
distance to the nearest wall in `grid_map.likelihood_field`, score each beam

    p = z_hit * exp(-0.5 * (d / sigma_hit)^2) + z_rand / range_max

and sum the logs. Beams outside the map get `max_dist`: penalised, and never an index error.
K is vectorised, not looped: the function runs N times per step.

Only hit beams come in. `step()` removes max-range returns before it calls this.

## `resampling.py` (Task 4)

**`effective_sample_size(weights)`**: `1 / sum(w^2)`. N for equal weights, 1 when one particle
has all the weight.

**`systematic_resample(weights, rng)`**: one random number, N positions 1/N apart, a lookup in the
cumulative sum. A particle with weight w is copied floor(N w) or ceil(N w) times. Less random
than N independent draws. `cumulative[-1] = 1.0` fixes rounding. `side='right'`: a particle with
weight 0 is never picked.

## `proposal.py` (Task 5)

**`proposal_candidates(x_star, cfg)`**. A regular 3-D lattice around the scan match `x*`. K is
rounded down to a cube: 27 gives 3 x 3 x 3. The windows are the **full** width: offsets from
-w/2 to +w/2. (The scan matcher windows are half-widths, +-w.)

**`improved_proposal(x_star, x_prev, u, endpoints, grid_map, cfg)`**. For each candidate:

    log tau_j = log p(z | x_j, m) + log p(x_j | x_prev, u)

`log eta = logsumexp(log tau)` is the particle's weight factor (a K-point estimate of
p(z | x_prev, m, u)). The normalised tau give a weighted mean (heading through sin and cos) and a
weighted covariance (heading difference wrapped). Then a variance floor `SIGMA_REG` and an
eigenvalue floor `SIGMA_EIG_MIN`, so Sigma stays a valid covariance when all weight is on one
candidate. Everything is in log space: a product of hundreds of beam probabilities underflows.

## `rbpf.py`, `step()` (Task 6)

Before the loop: split the beams once. `hits` are ranges below `range_max`; a max-range return
maps free space but must not score a pose. `ep_match` and `ep_w` are every Nth hit (strides 1).

For each particle:

1. (3) `x_bar` from the motion model.
2. (5) FS2: `scan_match(x_bar, p.grid, ep_match, cfg)` against **this particle's own map**.
   FS1: no match, score `-inf`.
3. If `score >= match_score_min`: (6-10) `improved_proposal`, (11) sample the new pose from
   N(mu, Sigma), wrap the heading, (12) add `log_eta` to the weight.
4. Else (fallback): keep `x_bar`, add the plain log-likelihood. Count it in `n_fallback` when FS2.
5. (22) `integrate_scan` at the new pose, **last**, so every likelihood above saw m(t-1).

After the loop: (24-25) `_normalize` gives weights and N_eff, `best_index` is read before the
resample, (26-27) `_maybe_resample`. The returned N_eff is the one **before** resampling.

Given parts of `rbpf.py` we did not change: `_normalize` (log space), `_maybe_resample` (copies
the grid of each survivor; a shared grid would merge hypotheses), `best_particle()` (largest
log weight).

---

## `config/params.yaml`: our values and why

| Value | Shipped | Ours (sim) | Why |
|---|---|---|---|
| `map_resolution` | 0.01 | 0.05 | 1 cm: no walls form, and 100 MB per particle. 5 cm: walls appear (Task 1) |
| `map_size_m` | 50 | 20 | 0.6 MB per particle. The maze fits |
| `p_occ` | 0 | 0.7 | +0.85 log-odds per hit. 0 makes a hit count as free |
| `p_free` | 0 | 0.4 | -0.41 per pass, weaker than a hit: a grazing beam cannot erase a wall |
| `hit_window_cells` | 0 | 2 | +-5 cm occupied band. 0 gives no walls |
| `log_odds_limit` | 0 | 5 | 0 clamps every cell to unknown. 5 caps p at 0.993 |
| `odometry_sigmas` | 0 | [0.05, 0.04, 0.05] | Wider than the odometry noise in one 0.2 m step (0.033 rad, 0.023 m at 0.003) |
| `odom_noise_sigma` | 0 | 0.003 (sweep), 0 (tuning, hall) | PDF 4.1. Hall: 0, real odometry has real noise |
| `scan_match_window_xy` / `_theta` | 0 | 0.2 / 0.1 | Half-widths. Last step 5 cm = one cell, 0.025 rad |
| `match_score_min` | 0 | 0.55 | An empty map scores 0.5. 0.3 disabled the fallback |
| `num_candidates` | 3 | 27 | 3 rounds to 1 candidate: Sigma is only the floor |
| `proposal_window_xy` / `_theta` | 0 | 0.1 / 0.05 | Full width: +-5 cm, +-0.025 rad, the matcher's last step |
| `resample_threshold` | 0 | 0.5 | Selective resampling, N_eff < N/2 |
| `resample_every_step` | true | false | Resampling on every step throws diversity away |
| `update_min_dist` / `_angle` | 0.05 / 0.0175 | 0.2 / 0.2 | 559 steps gave a heading random walk; 136 steps fixed it |
| `scan_queue_depth` | 1 | 1000 (sim), 1 (hall) | Replays must process every scan |
| `ground_truth_topic` | '' | /ground_truth (sim), '' (hall) | Gazebo /odom drifts 0.17 m; it is not the truth |

---

## The given node, the parts you must know

- **Gate** (`_gated`): a scan drives a step only after 0.2 m or 0.2 rad, and at most
  10 Hz. A parked robot does not integrate the same scan again.
- **Noise injection** (`_corrupt`, sim only): `odom_noise_sigma` is added to **each** /odom
  message (44 Hz in Gazebo), also when the robot stands still. It does not use our motion model
  on purpose: a bug in our code must not change the experiment.
- **One random generator** (seeded by `seed`) for the noise and the filter. A change in the
  filter changes the noise sample. That is why the sweep needs seeds.
- **TF**: publishes map -> odom so that map -> odom -> base is the best particle's pose.
- **Run log**: on Ctrl-C, `runs/<run_name>.npz` (estimate, odometry, truth, N_eff, resampled,
  fallbacks, three timings, best index, map bytes, final map) and `.json` with every value.
