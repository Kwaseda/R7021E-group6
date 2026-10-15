# Lab 4 run sheet

Robot: turtle ___   IP 192.168.50.__0   domain 3__
Mapping: turtleN -> 192.168.50.N0 -> domain 3N. Example: turtle 4 is 192.168.50.40, domain 34.

**Read the number on the robot first.** Do not trust the domain in `~/.bashrc`. Set it by hand in
every laptop terminal and again in the SSH terminal on the robot.

Nothing here needs the internet or Claude. The hall router is a local network. If a step fails,
use "When it breaks" at the bottom.

Hall session: Fri 16 Oct, 13:00 to 18:00. Tasks 8 and 9.

---

## Before the hall (at home, with internet)

1. `git pull` in `~/R7021E-group6`.
2. Build once:

```bash
source /opt/ros/jazzy/setup.bash
```

```bash
cd ~/R7021E-group6/lab4/ros2_ws && colcon build --packages-select r7021e_fast_slam --symlink-install
```

You must see `Summary: 1 package finished`.

3. Check that `params.yaml` is in hall mode. The last commit before the hall set it:

```bash
grep -E "odom_noise_sigma|scan_queue_depth|ground_truth_topic|update_min|odometry_sigmas" ~/R7021E-group6/lab4/ros2_ws/src/r7021e_fast_slam/config/params.yaml
```

You must see `odom_noise_sigma: [0.0, 0.0, 0.0]`, `scan_queue_depth: 1` and `ground_truth_topic: ''`.
The tuned values stay: `odometry_sigmas: [0.05, 0.04, 0.05]`, `update_min_dist: 0.2`,
`update_min_angle: 0.2`.

4. Test the screen recorder one time (it has not been tested on this laptop). Press `Print`.
In the bar that opens, click the video camera icon, choose "Window", click the RViz window,
then the round red button. To stop, click the red button in the top bar. The file goes to
`~/Videos/Screencasts/`.

---

## The stop command

Paste this in its own terminal before anything moves. Leave it there:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{}"
```

The robot keeps its last command. If something is wrong: up arrow, Enter. Then Ctrl-C the teleop.

## Every laptop terminal starts with these lines

Replace `34` with 3 times the robot number.

```bash
source /opt/ros/jazzy/setup.bash
```

```bash
export TURTLEBOT3_MODEL=burger
```

```bash
export ROS_DOMAIN_ID=34
```

```bash
source ~/R7021E-group6/lab4/ros2_ws/install/setup.bash
```

```bash
cd ~/R7021E-group6/lab4/ros2_ws/src/r7021e_fast_slam
```

```bash
printenv ROS_DOMAIN_ID
```

The same number in every terminal. A wrong domain gives an empty ROS graph and no error.
All commands below run from the package folder, so `runs/` is the right folder.

---

# In the hall

Open six terminals. Name them in this order.

### Terminal 1: robot bringup (SSH, on the robot)

```bash
ssh turtle@192.168.50.40
```

Password `turtle`. On the robot:

```bash
source /opt/ros/jazzy/setup.bash
```

```bash
export TURTLEBOT3_MODEL=burger
```

```bash
export ROS_DOMAIN_ID=34
```

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

It prints lines about the motors and the lidar and keeps running. Leave it.

### Terminal 2: checks (laptop)

Set the robot clock over the hall Wi-Fi (no internet needed). Without this, TF rejects the
messages:

```bash
ssh -t turtle@192.168.50.40 "sudo date -u -s '$(date -u +%Y-%m-%dT%H:%M:%S)'"
```

```bash
ros2 topic list
```

You must see `/scan`, `/odom`, `/tf`, `/cmd_vel`. A short list means a wrong domain.

```bash
ros2 topic info /cmd_vel -v
```

`TwistStamped`, and `Subscription count: 1`.

Prove the robot moves without our code:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.05}}}"
```

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{}"
```

### Terminal 3: the stop command

Paste it. Do not press Enter.

### Terminal 4: the filter

Put the robot on the start line first. The filter takes the first pose as the origin.

```bash
ros2 launch r7021e_fast_slam r7021e_fast_slam.launch.py num_particles:=10 run_name:=hall_tune_1
```

No `use_sim_time` on the robot. RViz opens with the particle cloud, the path and the map. The
node writes `runs/<run_name>.npz` and `.json` when you press Ctrl-C **once**. Wait for
`run log written to`.

### Terminal 5: teleop

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Drive slowly: `w` two or three times (about 0.1 m/s), slow turns, `s` to stop. Do not touch walls.

### Terminal 6: the bag

Start it before the robot moves. A new name for each run:

```bash
ros2 bag record /scan /odom /tf /tf_static -o runs/hall_run1
```

Ctrl-C **once** after the run, and wait for `Recording stopped`. Then:

```bash
ros2 bag info runs/hall_run1
```

Duration and message counts must not be 0.

---

## The order of work in the hall (5 hours)

| Time | What | Run names |
|---|---|---|
| 13:00 | Terminals 1 to 3. Robot moves without our code | |
| 13:20 | Task 8 tuning: short drives, one change at a time (table below) | `hall_tune_1`, `hall_tune_2`, ... |
| 15:00 | Task 8 run 1: full maze, back to the start. Bag and screen recording | `hall_run1` |
| 15:40 | Task 8 run 2: two loops, back to the start. Bag and screen recording | `hall_run2` |
| 16:30 | Task 9: wheels in the air. Bag and screen recording | `hall_wheels` |
| 17:00 | Notes in `LOG.md` while you are still in the hall. Commit | |

For each tuning run: Ctrl-C the filter once, read the numbers, change **one** value in
`config/params.yaml` with a text editor, save, start the filter again with a new `run_name`.
The node reads `params.yaml` only at start. No build is needed (`--symlink-install`).

Numbers after each run (no ground truth on the robot):

```bash
python3 ~/R7021E-group6/lab4/desk/hall_summary.py runs/hall_*.npz
```

"end gap" is how far the last estimate is from the start. Drive back to the start, so small is
good. Also look at RViz: one sharp wall, or two walls side by side?

## Task 8 tuning: what to change, in what range, and why

Change one value at a time. Start from the values in `params.yaml`. Go to the next row only
when the current row looks right.

| Order | Value | Now | Try between | Look at | Too small | Too large |
|---|---|---|---|---|---|---|
| 1 | `num_particles` (launch arg) | 10 | 5 to 30 | step ms p95 against about 200 ms | Cloud has no choice | Steps slower than the scan, filter lags |
| 2 | `odometry_sigmas` | [0.05, 0.04, 0.05] | rot 0.03 to 0.10, trans 0.02 to 0.06 | Double walls after turns; cloud size | Pose lags in turns, double walls | Cloud wide, pose shakes |
| 3 | `scan_match_window_theta` | 0.1 | 0.05 to 0.2 | Heading after turns | Cannot correct a turn error | Slow, can lock on a wrong wall |
| 4 | `scan_match_window_xy` | 0.2 | 0.1 to 0.3 | Map after a long straight | Cannot correct slip | Slow, wrong peaks |
| 5 | `match_score_min` | 0.55 | 0.55 to 0.75, never 0.5 or less | fallbacks count | Trusts bad matches | Falls back too often, odometry only |
| 6 | `resample_threshold` | 0.5 | 0.3 to 0.7 | neff/resam | Weights pile on one particle | Resamples every step, cloud collapses |
| 7 | `update_min_dist`, `update_min_angle` | 0.2, 0.2 | 0.1 to 0.3 each | steps count, drift | Many steps, small errors add up | Big jumps per step, needs wider sigmas |
| 8 | `p_occ` (map only) | 0.7 | 0.65 to 0.8 | Thin or broken walls | Walls appear slowly | One stray return makes a wall |

How the values depend on each other:

- **Step size and motion noise.** A bigger step (`update_min_dist`, `update_min_angle`) holds
  more odometry error, so `odometry_sigmas` must be wider. A smaller step needs smaller sigmas.
- **Motion noise and the matcher.** `odometry_sigmas` is also the matcher's prior. Wider sigmas
  let the scan pull the pose further. The windows must be about 3 times the sigmas, or the true
  pose is outside the search.
- **Matcher and map.** The matcher's finest step is a quarter of its window. Keep it near one
  cell (0.05 m), so `scan_match_window_xy` 0.2 fits `map_resolution` 0.05.
- **Proposal and matcher.** `proposal_window_xy` 0.1 (offsets +-0.05 m) and
  `proposal_window_theta` 0.05 must cover the matcher's last step. Change them together with the
  matcher windows, not alone.
- **N and time.** Each particle has its own map. Time and memory grow with N. If the step takes
  longer than the scan period, the node drops scans (`scan_queue_depth` 1 on the robot).
- **`match_score_min` and the map.** An unknown cell is p = 0.5, so an empty map scores 0.5. The
  threshold must stay above 0.5, or the fallback never runs.

On the real robot the odometry is better than in our simulation (the heading uses the IMU), so
smaller `odometry_sigmas` can be right there. That is the main thing to test.

## Task 8 runs

1. **Run 1:** full maze, back to the start line. Terminal 6 `-o runs/hall_run1`, filter
   `run_name:=hall_run1`, screen recording on.
2. **Run 2:** two loops through the maze, back to the start. `hall_run2`.

Write in `LOG.md` with [hall]: what you changed from the simulation and why, the end gap,
what the map looked like, anything different from Gazebo.

## Task 9: wheels in the air

Put the robot on the support so the wheels turn freely. Start the filter with
`run_name:=hall_wheels`, the bag with `-o runs/hall_wheels`, and the screen recording. Teleop
forward and turn for about 30 s. Watch the particle cloud, the pose and the map.
Write what you see in `LOG.md` at once, [hall]. The explanation is yours (`STUDY/ORAL.md`).

## Ending a run

1. Teleop: `s`, then Ctrl-C.
2. Terminal 6: Ctrl-C once, wait for `Recording stopped`.
3. Terminal 4: Ctrl-C once, wait for `run log written to`.
4. Stop the screen recording.
5. `ros2 bag info runs/<name>`.

## After the hall

```bash
cd ~/R7021E-group6 && git add -f lab4/ros2_ws/src/r7021e_fast_slam/runs/hall_*.npz lab4/ros2_ws/src/r7021e_fast_slam/runs/hall_*.json lab4/LOG.md lab4/ros2_ws/src/r7021e_fast_slam/config/params.yaml
```

```bash
git commit -m "Lab 4 hall runs and notes"
```

Push at home (`git push`); the hall has no internet.

`runs/` is ignored by git, so `-f` is needed. The bags are large and stay out of git. Copy them and the screen recordings to a USB stick for the report.

---

# At home: Gazebo (already done for Tasks 1 and 7)

The sim work is finished. The drive bag is `runs/maze_drive`. To replay one run:

```bash
ros2 launch r7021e_fast_slam r7021e_fast_slam.launch.py use_sim_time:=true use_rviz:=true run_name:=replay_test
```

```bash
ros2 bag play runs/maze_drive --clock
```

Sim values that differ from the hall: `odom_noise_sigma: [0.003, 0.003, 0.003]`,
`scan_queue_depth: 1000`, `ground_truth_topic: /ground_truth`, `use_sim_time:=true`.

---

# When it breaks

| What you see | Check | Fix |
|---|---|---|
| `ros2 topic list` almost empty | `printenv ROS_DOMAIN_ID` | Same domain in every terminal and on the robot |
| Robot does not move from teleop | `ros2 topic info /cmd_vel -v` | Both ends `TwistStamped`, bringup running |
| RViz: no map, "no transform", `TF_OLD_DATA` | Clock on the robot | Clock command in Terminal 2 |
| RViz shows no laser dots | LaserScan display | Reliability Policy: Best Effort. Fixed Frame: `map` |
| Node does nothing, warning about `/clock` | Launch line | No `use_sim_time` on the robot |
| Map has two walls side by side | Turns too fast, or sigmas too small | Slower turns. Then row 2 of the tuning table |
| Pose jumps along a corridor | fallbacks count | A long corridor looks the same everywhere. Drive past a corner |
| Bag is 0 s | Domain in Terminal 6 | Start the bag after the setup lines |
| `run log` not written | Pressed Ctrl-C twice | Press once and wait |
