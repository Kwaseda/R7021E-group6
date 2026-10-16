# Lab 4 report notes

Facts for the group report, task by task. Every number has a source: [desk], [gazebo] or
[hall], and the details are in `LOG.md`. The report itself is written in your own words.

PDF section 9: one report per group, with all figures, observations and discussions from the
tasks that are not only code. Section 8: submit the code, the bags and RViz screen recordings
of Tasks 8 and 9, and the `params.yaml` used.

---

## Submission checklist

| Item | Where |
|---|---|
| Code: the five student files | `ros2_ws/src/r7021e_fast_slam/r7021e_fast_slam/` (`motion_model.py`, `measurement_model.py`, `resampling.py`, `proposal.py`, `rbpf.py` `step()`) |
| `params.yaml` used in the hall | `ros2_ws/src/r7021e_fast_slam/config/params.yaml`, and the `.json` of each hall run |
| Task 8 run 1 bag | `runs/hall_run_official_1_nasser` |
| Task 8 run 2 bag | `runs/hall_run_official_2_nasser` |
| Task 9 bag | `runs/hall_wheels_up_nasser` |
| Task 8 run 1 screen recording | `~/Videos/Screencasts/Screencast from 2026-10-16 14-39-02.webm` |
| Task 8 run 2 video (bag replay, no screen recording was made) | `~/Videos/lab4/task8_run2_replay.mp4` |
| Task 9 screen recording | `~/Videos/Screencasts/Screencast from 2026-10-16 14-54-53.webm` |
| Replay videos of run 1 and Task 9 (backup) | `~/Videos/lab4/task8_run1_replay.mp4`, `task9_wheels_replay.mp4` |
| Figures | `ros2_ws/src/r7021e_fast_slam/figures/` (Task 1, the four of Task 7, `hall_maps.png` for Tasks 8 and 9) |

The bags and videos are not in git (size). Copy them to the submission.

How the replay videos are made: `video/replay_record.sh` plays the hall bag through the same
filter with the same values (from the run's `.json`) and records `/map`, `/particles`,
`/slam_path` and `/scan`. `video/bag_to_video.py` draws them like RViz. A replay draws new
random numbers, so it is the same filter on the same sensor data, not a copy of the live run.
92 % of the scans in the run 2 replay land on the map walls; the rest are short moments in
turns, between two filter steps.

---

## Tasks 2 to 6: the code

Code only, nothing to report. One sentence each if you want it:

- Task 2 `motion_model.py`: sample from the odometry model (rot1, trans, rot2), and its log density for candidate poses. Both forms use the same noise.
- Task 3 `measurement_model.py`: likelihood field. p = z_hit exp(-d^2 / 2 sigma_hit^2) + z_rand / range_max per beam, sum of logs.
- Task 4 `resampling.py`: N_eff = 1 / sum(w^2); systematic resampling with one random number.
- Task 5 `proposal.py`: 27 candidates on a 3 x 3 x 3 lattice around the scan match, tau = likelihood x motion density, eta = sum of tau is the weight, Gaussian fit to the weighted lattice.
- Task 6 `rbpf.py` `step()`: predict, scan match, proposal or fallback, weight, map update last, normalise, resample when N_eff < N/2.

Desk check of all five before Gazebo: all pass [desk] (LOG 2026-10-13).

---

## Task 1: occupancy mapping with known poses

Figure: `figures/maps_task1.png` (`mapping_shipped` against `mapping_tuned`, same bag).

(a) What we changed, why, and the effect, one value at a time [gazebo]:

| Step | Change | Effect |
|---|---|---|
| shipped | none | 100 % unknown map: `log_odds_limit` 0 clamps every update back to 0 |
| 1 | `log_odds_limit` 0 to 5 | Cells can change, but `p_occ` 0 makes a hit add free evidence: no walls |
| 2 | `p_occ` 0 to 0.7 | Still no walls: `hit_window_cells` 0 is a band of zero width |
| 3 | `p_free` 0 to 0.4 | Free evidence weaker than hit evidence |
| 4 | `hit_window_cells` 0 to 2 | A few walls (0.63 m2). At 1 cm the noisy endpoints seldom hit one cell two times |
| 5 | `map_resolution` 0.01 to 0.05, `map_size_m` 50 to 20 | Walls appear (5.2 m2). Resolution was the key value. Memory 100 MB to 0.6 MB per particle |
| 6 | `use_clamping` false | Almost no change on a 3.5 min drive |

The tuned map still has double walls, turned against each other: it uses raw corrupted
odometry (pose error 0.47 m mean). Task 7 fixes that.

Limitation to state: the node draws noise per received /odom message, so the slow 1 cm run
saw a different noisy trajectory (1.64 m against 0.47 m mean error).

---

## Task 7: evaluation in simulation

Set-up: one drive bag `runs/maze_drive` (218 s, 13.9 m, `lab3_maze_small`), replayed for every
run. Ground truth is Gazebo's true pose (`/ground_truth`): Gazebo's own /odom was 0.17 m off the
true pose, so it is not usable as truth [gazebo].

Tuning on the bag (FS2, N = 10), the two changes that mattered [gazebo]:

1. Filter step every 0.2 m or 0.2 rad, not every 0.05 m or 1 degree. 559 steps gave ATE 0.309 m (worse than odometry); 136 steps gave 0.085 m. Each step leaves a small heading error that the 5 cm scan matcher cannot remove, and the errors add up.
2. `odometry_sigmas` [0.05, 0.04, 0.05]. A 0.2 m step holds 0.033 rad and 0.023 m of the injected noise; 0.02 was too narrow (ATE 0.452 against 0.343 m raw); the wider values gave 0.196 against 0.579 m raw.

Earlier fix: `match_score_min` 0.3 to 0.55, because an empty map scores 0.5.

Figure 1, `figures/maps_task7.png`: raw odometry map has double walls turned by about 0.3 rad; the FS2 map (N = 20) has single walls and the estimate on the true path (ATE 0.070 m).

Figure 2, `figures/ate_vs_particles.png`, ATE rmse, 3 seeds, mean (min to max) [gazebo]:

| N | FS1 | FS2 |
|---|---|---|
| 1 | 1.081 (0.636 to 1.683) | 0.387 (0.272 to 0.476) |
| 5 | 0.342 (0.184 to 0.545) | 0.239 (0.199 to 0.279) |
| 10 | 0.210 (0.137 to 0.283) | 0.130 (0.088 to 0.197) |
| 20 | 0.175 (0.082 to 0.324) | 0.161 (0.070 to 0.326) |
| 50 | 0.101 (0.085 to 0.131) | 0.142 (0.121 to 0.180) |

Discussion points: FS2 is about 3 times better at N = 1 and has narrower bands; both drop with
N; from N = 10 the bands overlap. FS2 needs fewer particles (Lecture 9). The bands are wide
because the node uses one random generator for the noise and the filter, so each run sees a
different noise sample.

Figure 3, `figures/neff.png` (N = 10): selective resampling still resampled 78 % of the steps,
every-step 100 %; median N_eff / N 0.34. The likelihood is sharp (about 300 beams, each taken as
independent), so the weights collapse most steps. A flatter likelihood (`sigma_hit` up, or
`likelihood_stride` 2 to 4) would give a larger difference.

Figure 4, `figures/timing.png`: time per step linear in N, about 35 ms at N = 1, 230 ms at
N = 10, 830 ms at N = 50; scan match and proposal dominate, map update small. Memory 0.64 MB per
particle, 32 MB at N = 50. A step must end before the next gated scan (about 0.4 s in a turn),
so N = 50 falls behind on the robot.

---

## Task 8: transfer and tune on the robot [hall]

All hall runs: FS2, N = 10, `odom_noise_sigma` 0, `scan_queue_depth` 1, no ground truth.
(The notes from the hall say N = 20 for two runs, but every run log says 10, and the step time
did not change. Write N = 10.)

Tuning runs:

| Run (bag) | Change | Steps | End gap est / odom | Fallbacks per step | Step ms median |
|---|---|---|---|---|---|
| `hall_tune_1_nasser` (`hall_run1_nasser`) | sim values | 156 | 0.44 / 0.36 m | 0.06 | 76 |
| `hall_tune_2_nasser` (`hall_run2_nasser`) | same values | 101 | 0.09 / 0.09 m | 0.10 | 77 |
| `hall_tune_3_nasser` (`hall_run3_odom_sigmas_nasser`) | `odometry_sigmas` [0.03, 0.02, 0.06] | 82 | 0.11 / 0.21 m | 0.12 | 76 |

Official runs, `odometry_sigmas` [0.03, 0.02, 0.06]:

| Run | Bag | Steps | Time | End gap est / odom |
|---|---|---|---|---|
| Run 1, full maze | `hall_run_official_1_nasser` | 93 | 282 s | 1.90 / 2.01 m |
| Run 2, two loops | `hall_run_official_2_nasser` | 146 | 320 s | 0.38 / 0.42 m |

"End gap" is the distance of the last pose from the start. Run 1 did not end at the start:
the robot's own odometry also puts it 2.0 m away, so the robot was stopped before it was back.
State this. In run 2 the robot stopped about 0.4 m from the start line.

Observations for the report:

- All maps have single, sharp walls (`figures/hall_maps.png`, and the videos). The maze is about 4.5 x 2.5 m.
- The estimate stays close to the raw odometry (end gaps differ by 0.04 to 0.12 m). The real wheel odometry, with the IMU heading, is much better than our simulated odometry with noise 0.003, which drifted 0.2 to 1 m.
- Fallbacks are rare (0.1 of 10 particles per step): the scan match is trusted almost every step.
- Step time 76 ms at N = 10 against 230 ms in simulation, far under the time between steps.
- Changing N and `odometry_sigmas` gave no visible change. With good odometry, every particle is already near the truth, so the filter has little to correct. This is the main difference from simulation.
- Tuning changed on hardware: `odometry_sigmas` [0.05, 0.04, 0.05] to [0.03, 0.02, 0.06] (smaller translation and rot1 noise, because the real odometry is better; rot2 a little larger. Say your own reason). Nothing else.

---

## Task 9: wheels in the air [hall]

Run `hall_wheels_up_nasser`, 163 s, 46 filter steps, N = 10.

What the data shows:

- The odometry says the robot drove 9.3 m back and forth, and ended 1.72 m from the start. The robot did not move.
- The estimate followed: 8.8 m of path, 1.40 m from the start. On each step it moved 95 % (median) of the odometry step.
- The map smeared: the same walls were painted again at each new pose, so they became long streaks along the drive direction.
- Fallbacks: 5.5 of 10 particles per step (0.1 when driving normally). On 13 of 46 steps all 10 particles fell back.
- N_eff 3.5 of 10, resampled on 38 of 46 steps.

The hall note says "nothing much really". The run log shows the opposite: the filter believed
the wheels. Correct the note in the report.

Why, by stage of the algorithm (step numbers of the lab PDF):

1. Gate: the gate measures distance on the odometry. The odometry says 0.2 m, so a step runs.
2. Step 3: every particle moves 0.2 m forward from the motion model.
3. Step 5: the scan matcher searches around that moved pose. Its prior holds it near the odometry prediction: moving back 0.2 m costs 0.1 x 0.5 x (0.2 / 0.02)^2 = 5 in the selection score, but the score is at most 1. So the match stays near the moved pose.
4. The match score there is low (the walls are 0.2 m off), so for about half the particles it is below 0.55 and they fall back: they keep the odometry pose and are weighted by the likelihood alone.
5. Steps 6 to 12 (the other particles): the lattice is only +-5 cm around the match, so it cannot reach the true pose 0.2 m back either.
6. Weights and resampling only choose among particles that all moved with the odometry. The selection cuts about 5 % of each step, not 100 %.
7. Step 22: the scan is added to the map at the wrong pose, so the map smears.

In one sentence: our model says the odometry error per step is about 2 cm, so a 20 cm lie is
outside what the filter thinks is possible, and the laser cannot win.

Comparison (PDF asks for it):

- FastSLAM 1.0: samples only from the motion model, so every particle moves with the odometry, spread by `odometry_sigmas`. The weights pick the particle that moved least, but all moved about 0.2 m. Same result, without the matcher.
- EKF-SLAM: one Gaussian. The prediction moves the mean with the odometry and grows the covariance. The update compares the predicted landmark positions with the observed ones and pulls the mean back by the Kalman gain. The gain grows as the covariance grows, so it would correct part of each step. It would still drift if Q (odometry noise) is small against R (sensor noise), and a wrong data association can make it diverge.
