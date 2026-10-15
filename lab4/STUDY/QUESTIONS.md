# Lab 4 questions

Answer each one out loud in two sentences, with no notes. The answers are in the study guide
(`Lab 4 Study Guide.html`), `CODE.md`, `LOG.md` and the code. This file does not hold them.

How to use it:

1. Read one part of the study guide. Close it.
2. Answer that part's questions here. "I don't know" is a good answer. Then look it up.
3. Two days later, answer the same questions again before you open anything.
4. A wrong answer comes back after 1, 3 and 7 days.

---

## Q1. Occupancy grids (Task 1)

1. What does a log-odds value of 0 mean? Why is the Bayes update then a plain sum?
2. `p_occ` 0.7: what is the log-odds increment? And for `p_free` 0.4?
3. Why must the free increment be weaker than the hit increment?
4. The shipped map was 100 % unknown. Which value caused that, and how?
5. Why did 1 cm cells give no walls, and 5 cm cells give walls?
6. What does `hit_window_cells` do, and how does it interact with the resolution?
7. Why did the tuned Task 1 map still have double walls?

## Q2. Motion model (Task 2)

1. What are rot1, trans and rot2? Why does rot1 become 0 when the robot barely moves?
2. What does `sample_motion_model_odometry` return, and for what step of the algorithm?
3. Why does FS2 also need the density form?
4. What happens if the sampler and the density use different noise?
5. Why are the angle residuals wrapped?

## Q3. Measurement model (Task 3)

1. Write the beam probability. What is each term for?
2. Why sum logs and not multiply probabilities?
3. What does a beam that lands outside the map get, and why?
4. Why are max-range returns not used to score a pose?
5. Why must K be vectorised and not looped?

## Q4. Resampling (Task 4)

1. N_eff for equal weights? For all weight on one particle?
2. How does systematic resampling place its N positions? Why is it better than N draws?
3. Why resample only when N_eff < 0.5 N?
4. Why does the log store N_eff before the resample?
5. What does the resample copy, and why must the grid be copied?

## Q5. Improved proposal (Task 5)

1. Why is a single scan-matched pose not a proposal?
2. What is tau_j? What is eta, and why is eta the weight?
3. Why is the heading mean computed with sin and cos?
4. Why is K rounded to a cube? What happens at K = 3?
5. Full-width proposal windows against half-width matcher windows: what are the offsets for
   `proposal_window_xy` 0.1 and `scan_match_window_xy` 0.2?
6. Why the covariance floors?

## Q6. The filter (Task 6)

1. Give the order of one step for one particle, with step numbers.
2. Why is the map integrated last?
3. When does the fallback run? What weight does it give?
4. Why does an empty map score 0.5? What did 0.3 do to our filter?
5. Which particle is the estimate?

## Q7. Evaluation (Task 7)

1. What is ATE? Why is there no alignment?
2. Why one bag replayed many times, and not many drives?
3. Why three seeds? What does the shared random generator do?
4. Why was Gazebo /odom not the ground truth? What did we use?
5. Why did 559 steps give a worse FS2 than 136 steps?
6. Why did `odometry_sigmas` have to grow when the step grew?
7. What does each of the four figures show?

## Q8. Hardware (Tasks 8 and 9)

1. Which four values change from simulation to the robot?
2. How do you judge a run without ground truth?
3. Which knob first if the map has double walls after turns? Which if the node lags?
4. Wheels in the air: which stage decides what the filter believes?
5. How would FS1 and EKF-SLAM behave in the same test?
