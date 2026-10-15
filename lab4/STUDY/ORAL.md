# Lab 4 oral

The individual talk comes after the report is in (TA Nikolaos Stathoulopoulos, niksta@ltu.se).
Expect questions on any line of the five student files and on any value in `params.yaml`. This
file holds headings, an order and the knob behaviours. It does not hold the sentences you say.
You write those, from memory, and check them against the code.

---

## The opening: five sentences

Write one sentence under each heading, on paper, without notes. Then say them out loud two times.

1. The loop. What happens to one particle in one step, in order (steps 3, 5, 6 to 12, 22)?
2. FS1 against FS2. Where does the scan enter in each?
3. The weight. Why is it eta, and what is eta an approximation of?
4. One bug we had and how we found it (the 0.3 threshold, or the 559 steps).
5. One result from the hall, with a number.

A good sentence has a name or a number from the code or from `LOG.md`.

## Walk through the code

Be ready to open each file and read it aloud, top to bottom:

- `motion_model.py`: the two forms, and why they must agree.
- `measurement_model.py`: the mixture, the clip at `max_dist`, beams outside the map.
- `resampling.py`: the two ends of N_eff, one random number in systematic resampling.
- `proposal.py`: K to a cube, full-width windows, tau, eta, mu with sin and cos, the floors.
- `rbpf.py` `step()`: beam split, matcher, threshold, fallback, map last, N_eff before resample.

## What each knob does

Learn this table. The examiner can name a knob and ask what changes.

| Knob | Bigger | Smaller | Coupled with |
|---|---|---|---|
| `num_particles` | More hypotheses, better odds to hold the truth; time and memory grow linearly | Fewer choices; at N = 1 there is no filter | Step time against the scan period |
| `odometry_sigmas` | Wider cloud; weaker matcher prior, the scan pulls further; more noise per step | Stiff: the truth outside the cloud, double walls after turns | Step size, matcher windows |
| `update_min_dist`, `_angle` | Fewer steps, less error added up; larger odometry error per step | Many steps; a small error per step becomes a heading drift | `odometry_sigmas` must cover one step |
| `scan_match_window_xy`, `_theta` | Can correct bigger errors; more cost; can lock on a wrong wall | Cannot reach the true pose | About 3 x `odometry_sigmas`; last step = window / 4 near one cell |
| `match_score_min` | More fallbacks: odometry and weight only | Trusts bad matches. At 0.5 or less never falls back | Empty map scores 0.5 |
| `num_candidates` | Finer proposal lattice; cost grows with K | Below 8 it is one point and Sigma is only the floor | Proposal windows |
| `proposal_window_xy`, `_theta` | Lattice reaches poses the matcher rejected | Under one cell, all candidates read the same cells | Matcher last step |
| `resample_threshold` | Resamples more often; diversity lost | Weights pile on one particle | N_eff |
| `sigma_hit` | Flat likelihood, poses look alike, N_eff stays high | Sharp, N_eff collapses each step | `likelihood_stride` |
| `map_resolution` | Coarse walls, matcher steps coarse | Memory grows with 1/res^2; 1 cm gave no walls | `hit_window_cells` |
| `p_occ` / `p_free` | Faster walls / faster free space | Slower | A grazing beam must not erase a wall |
| `log_odds_limit` | Cells get sure, change slowly | 0 erases every update | `use_clamping` |

## The Task 7 figures: what to say

For each figure, have one sentence on what it shows and one on why. The numbers are in
`LOG.md`, entry "Task 7 figures".

1. Maps: raw corrupted odometry against FS2. Which walls are double, and why?
2. ATE against N: which proposal is lower at small N? Where do the lines meet? How wide are the
   bands, and what makes them wide?
3. N_eff against time: how often does each policy resample? What does every-step resampling do
   to the trace?
4. Time against N: which stage costs most? How does time grow with N? Memory?

## Task 9: wheels in the air

Explain it with the step numbers of the algorithm. Prompts, not answers:

- What does the odometry say? What does the laser say?
- Does the gate let steps through? What does it measure?
- Step 3: where does x_bar go? Step 5: where does the matcher search, and what score does it get
  on a map built at this spot? Above or below 0.55?
- Steps 6 to 10: what do the two factors of tau say? Which one is sharper?
- What would FS1 do? What would EKF-SLAM do with the same two inputs?

## Known weak points

Say these before you are asked:

- One random generator for noise and filter: runs at different N see different noise samples.
  Three seeds show the spread.
- Gazebo /odom is not the truth (0.17 m off). We recorded the true pose as `/ground_truth`.
- The scan matcher is coarse (5 cm, 0.025 rad). Too many steps add up its residual error.
- The tuning used Gazebo's own odometry (paired runs); the sweep uses 0.003 as the PDF asks.
