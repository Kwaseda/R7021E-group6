# Lab 4: why, and what the teacher may ask

Read `CODE.md` first. It has every function and value with its reason. The numbers and the story
of each decision are in `../LOG.md`.

## What the lab asks, and where it is

| Lab PDF | Ours |
|---|---|
| T1 occupancy mapping with known poses, shipped vs tuned map | `runs/mapping_shipped`, `runs/mapping_tuned`, `figures/maps.png`. Tuning steps in LOG |
| T2 motion model: sample and density | `motion_model.py` |
| T3 likelihood field measurement model | `measurement_model.py` |
| T4 N_eff and systematic resampling | `resampling.py` |
| T5 improved proposal: lattice, tau, eta, mu, Sigma | `proposal.py` |
| T6 the filter loop, map updated last, grids copied | `rbpf.py` `step()` |
| T7 tune, sweep N = 1, 5, 10, 20, 50 x 3 seeds, four figures | `params.yaml`, `runs/fs*_n*_s*`, `figures/` |
| T8 hall: tune, full maze, two loops, bags and screen recordings | `RUNSHEET.md` |
| T9 hall: wheels in the air, explain by algorithm stage | `RUNSHEET.md`, your own words |

The package is the course template. We did not change the node, the helpers or the tools.

## Questions to answer without notes

1. **Why a particle filter for SLAM?** The pose belief is not one Gaussian in a maze (many places look alike). Particles hold many hypotheses.
2. **What is Rao-Blackwellization here?** Sample only the path. Given a path, the map is "mapping with known poses", so each particle carries its own grid.
3. **FS1 vs FS2?** FS1 samples from odometry and lets the scan only weight. FS2 puts the scan into the sampling (proposal), so particles land where the scan fits and fewer are needed.
4. **Why is the weight eta, the sum over the lattice?** It approximates p(z | x_prev, m, u), the integral the weight should be. It is not the likelihood at the sampled pose.
5. **Why is the map integrated last?** Every likelihood in the step must see m(t-1). A map that already has the scan scores it too well.
6. **Why copy the grid on resample?** Two particles sharing one array would merge their hypotheses.
7. **Why does an empty map score 0.5?** Unknown cells are p = 0.5 and the score is the mean occupancy at the endpoints. So `match_score_min` must be above 0.5. We had 0.3 first and the filter got worse than odometry.
8. **Why selective resampling?** Resampling copies good particles and deletes the rest. Each time, diversity is lost. Do it only when N_eff < N/2.
9. **Why steps every 0.2 m / 0.2 rad?** Each FS2 step leaves a small heading error the 5 cm matcher cannot remove. More steps, more error added up. 559 steps gave ATE 0.309 m, 136 steps gave 0.085 m [gazebo].
10. **Why `odometry_sigmas` [0.05, 0.04, 0.05]?** They must cover the odometry error in one step. A 0.2 m step at 0.003 noise holds 0.033 rad and 0.023 m. Too narrow: the particles never cover the truth.
11. **odometry_sigmas vs odom_noise_sigma?** The first is the filter's belief per step (ours, tune it). The second is noise the node adds per /odom message, sim only, 0 on the robot.
12. **Why /ground_truth, not /odom?** Gazebo /odom is wheel odometry and was 0.17 m off the true pose on our drive. The true pose comes from Gazebo itself.
13. **Why seeds?** One random generator drives the noise and the filter. A different N or seed gives another noise sample. Three seeds show the spread.
14. **Why log space?** A product of 300 beam probabilities underflows to 0.

## Hall rules

- `odom_noise_sigma` [0, 0, 0], `scan_queue_depth` 1, `ground_truth_topic` '', no `use_sim_time`.
- Set the robot clock first. Keep the stop command ready. Ctrl-C once, then wait.
- Bag and screen recording for every Task 8 and Task 9 run. Save `params.yaml` with the runs.
