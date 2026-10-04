# Carried-over values

Transcribed by hand from `~/advanced-robotics-2026` on 2026-10-01. Nothing in this folder
loads this file — it is here so the numbers that cost lab time are not lost, and so that
when a value is typed as a `declare_parameter` default there is a record of where it came
from.

**Provenance column means:**
- `measured` — observed in Gazebo or on the robot, with a number behind it
- `published` — from the TurtleBot3 Burger spec sheet
- `derived` — follows from another value by arithmetic
- `chosen` — someone picked it; it worked; no measurement behind it
- `disputed` — the old repo contradicts itself, see the note

---

## Robot facts — TurtleBot3 Burger

| Value | Number | Provenance |
|---|---|---|
| max linear velocity | 0.22 m/s | published |
| max angular velocity | 2.84 rad/s | published |
| wheel radius | 0.033 m | published |
| wheel separation | 0.160 m | published |
| footprint length × width | 0.178 × 0.138 m | published |
| LDS-01/02 range | 0.12 – 3.5 m | published |
| LIDAR beams / rate | 360 beams, ~5 Hz | measured on the robot |

> Read `max_linear_velocity` off the robot's own `turtlebot3_node` before quoting 0.22 in
> the report. Firmware revisions have shipped different values.

## Lab 1 — position controller (NID)

| Parameter | Value | Provenance |
|---|---|---|
| control period | 0.05 s (20 Hz) | chosen — faster than the robot's mechanics, slower than the 5–10 Hz lidar |
| NID offset `L` (`a`) | 0.10 m | derived — at v capped 0.22, max ω the transform can ask is 0.22/0.10 = 2.2 rad/s, inside 2.84 |
| `k_p` position | 0.8 1/s | chosen — saturates until error < 0.275 m, then exponential with τ = 1.25 s |
| goal tolerance | 0.05 m | chosen — the Task A acceptance criterion |
| odom staleness timeout | 0.5 s | chosen |
| `cmd_vel` frame_id | `base_link` | required — never empty, a stamped message with no frame reads as the epoch |

`L` and `max_linear_velocity` are **not independent**. If you raise v, recheck that ω stays
under 2.84.

## Lab 1 — figure-8 — DISPUTED, measure your own

Gerono lemniscate: `x = cx + W·sin(s)`, `y = cy + H·sin(2s)`, `s = 2π·(t/T)`.

| Source | width | height | note |
|---|---|---|---|
| `lab1-guide.md` | 1.0 | 0.5 | prose |
| `trajectory.yaml` (last edit) | 0.5 | 0.25 | uncommitted change |
| run sheet measured extent | ±1.99 | ±0.98 | implies W=2.0 H=1.0 |

Three different figure-8s. **Pick from the arena you actually have and measure what you get.**

| Parameter | Value | Provenance |
|---|---|---|
| lap period | 50 s | derived — 6.097 m path at W=0.5/H=0.25 → mean 0.122 m/s, peak 0.175 m/s (80% of limit), peak path turn rate 0.40 rad/s |
| laps | 2 | chosen — one lap can be luck |
| start delay | 3.0 s | chosen — lets the controller drive onto the path first, so lap 1 is tracking not a transient |
| publish period | 0.05 s | chosen — matches the control period |
| measured lap time, sim | 126 s | measured — at the *larger* figure-8, not at lap_period 50 |
| measured extent, sim | x ∈ [−0.98, 0.98], y ∈ [−0.46, 0.46] over 2 laps | measured — at W=0.5/H=0.25 |

The turn rate the *path* asks for is not the turn rate the *controller* commands: NID divides
the lateral demand by `L`, so ω is larger whenever the robot is off the path. 0.40 rad/s is
a floor, not a peak.

Arena note: `box.launch.py`, which the tutorial names, was never supplied for this course.
`turtlebot3_dqn_stage1` was the stand-in — inner wall faces at ±2.35 m.

## Lab 1 — wall follower

| Parameter | Value | Provenance |
|---|---|---|
| follow side | right | arbitrary — depends which way the robot points at the start |
| wall setpoint distance | 0.5 m (sim) / 0.40 m (robot) | chosen — inside lidar's 0.12 m min, >2× the 0.178 m footprint |
| forward speed | 0.15 m/s (sim) / 0.10 m/s (robot) | chosen — headroom under 0.22 so the turn rate is not distorted by the clamp |
| `k_p` wall (distance) | 1.2 rad/s per m | chosen — gentle; undamped P wall-followers oscillate |
| `k_heading` wall (angle) | 1.0 rad/s per rad | chosen — distance alone cannot tell approaching from receding |
| max distance error | 0.5 m | chosen — bounds hardest turn to 1.5×0.5 = 0.75 rad/s |
| acquire turn gain | 1.0 1/s | chosen — needed because from mid-room every wall is beyond the proportional band |
| wall beam separation | 40° | chosen |
| wall beam window | 5° | chosen |
| max wall angle | 35° | **derived, load-bearing** — the two-beam fit is singular at 90° − separation = 50°. Near it, a beam that sails past a small obstacle and lands on the far wall reads as a near wall raking away, and the follower steers *into* the obstacle on a confident wrong number. This caused the tight orbits around the 0.15 m pillars in `turtlebot3_world`. Node caps at 0.8× the singular angle regardless. |
| front stop distance | 0.35 m | chosen — below this, stop forward motion and pivot |
| corner turn rate | 0.5 rad/s | chosen — a corner should be a controlled pivot the lidar can keep up with |
| side / front sector width | 60° / 60° | chosen |
| loop close tolerance | 0.3 m | **measured** — over 12 start headings offline: 0.2 m closes 12/12 with perfect odometry but only 8/12 at a realistic 2% yaw and scale error; 0.3 m closes 12/12 at every drift level. Use 0.2 only if the spec demands the gate itself be 0.2 — the *logged* error is what answers the task. |
| loop min distance | 2.0 m (sim) / 1.5 m box, 3.0 m room (robot) | chosen — ≈ half the expected lap; without it the test fires on tick one |
| lap settle: seconds / angle / distance | 0.6 s / 15° / 0.15 m | chosen — a robot pointing into the room sits at the right distance with the wrong heading; the lap reference must be a pose on the track it will pass again |
| loop close heading tolerance | 45° | chosen — loose enough for wall following, tight enough to reject the 180° an out-and-back gives |

Measured results, sim: wall distance held **0.382 ± 0.020 m** (min 0.339, max 0.484); loop
detected after **142.3 s**; stopped **0.27 m** from where it started.

## Lab 1 — the saturation finding (the headline result, keep it)

| | commanded v | commanded ω | outcome |
|---|---|---|---|
| no velocity limiting | 0.920 m/s | 19.95 rad/s | **flipped over in Gazebo** |
| with saturation | 0.220 m/s | 2.840 rad/s | reached the goal |

Rule of thumb from the gains: ω ≈ 10 × lateral error, since `k_p/L = 0.8/0.10 = 8`
(the run sheet quotes 0.5/0.05 = 10 for the other gain set). Either way a lateral error
over ~0.28 m already demands more than the Burger's 2.84 rad/s.

Saturate by scaling v and ω down by a **common factor**, so the direction of motion is
preserved. Clamping them independently bends the commanded path.

## Lab 2 — MPC

| Parameter | Value | Provenance |
|---|---|---|
| `t_step` | 0.1 s | measured — worst observed solve ~45 ms, so 100 ms is safe |
| `n_horizon` | 20 steps | derived — 2 s, or 0.44 m of travel at 0.22 m/s |
| goal tolerance | 0.05 m | chosen — tighter and it creeps on odometry noise |
| max linear velocity | 0.22 m/s | published |
| max angular velocity | **0.8 rad/s** | chosen — deliberately well under 2.84 for MPC |
| min linear velocity | 0.0 | chosen — no reversing |
| `q_position` / `q_terminal` | 1.0 / 1.0 | chosen |
| `r_input` | 0.01 | chosen — penalty on *changes* in v and ω, for smooth commands |
| obstacle inflation | 0.105 m | derived — the Burger's half-diagonal, since the model is a point |
| obstacle soft / penalty | false / 10000.0 | chosen |
| odom timeout | 0.5 s | chosen |
| boundaries, task 1 | x,y ∈ [−1, 1] | task spec |
| boundaries, task 2 | x,y ∈ [−2, 2]; obstacle at (0.75, 0.08) r = 0.30 | task spec |
| circle trajectory | radius 0.8, lap 60 s, 2 laps, start delay 8.0 s | chosen — 8 s to drive from the centre onto the circle |

Dependency pins that mattered: `casadi==3.7.2`, `do-mpc==5.1.1`, installed with
`pip install --user --break-system-packages`. The old `lab2-guide.md` explains why casadi
must be pinned exactly.

## Network

```
turtleN  ->  192.168.50.N0  ->  ROS_DOMAIN_ID 3N
```
Wi-Fi `turtlenet`, password from an instructor. SSH `ssh turtle@192.168.50.N0`.

**Unresolved:** `~/.bashrc` currently has `export ROS_DOMAIN_ID=32  # turtle4`. 32 is turtle2;
turtle4 is 34. Confirm the robot number on lab day and fix both the value and the comment.

## Emergency stop

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{}"
```

Keep it pasted in a terminal, ready for Enter. Every node you write must also publish zero
velocity on shutdown — the old unimproved nodes did not, which is why they were never run on
hardware.
