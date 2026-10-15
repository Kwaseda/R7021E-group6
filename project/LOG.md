# Project log

Newest entry at the bottom. What was done, what was decided, what went wrong.

---

## 2026-10-15: planning, skeleton, copy of Lab 2

### Decisions (planning interview with Dominic)

| # | Decision | Why |
|---|---|---|
| Q1 | New package `project/ros2_ws/src/r7021e_project`. Lab 2 code copied, not moved. | Lab 2 is graded and must still run as submitted. |
| Q2 | Plank 0.50 x 0.20 m, long axis along the heading, three discs at -0.167 / 0 / +0.167 m, r = 0.130. Single-disc baseline r = 0.269. | A plank across the heading cannot pass a 0.30 m gap under any model. Three is the smallest n that fits 0.30: two discs need 0.32 m. |
| Q3 | Plank centred on the wheel axle (`base_footprint`, the `/odom` point). | Symmetric offsets, simplest equations. |
| Q4 | Plank has collision in Gazebo, mounted above the lidar. Clearance comes from ground-truth pose against the exact rectangle. | Grading the disc model with discs would be circular. |
| Q5 | Box bounds become 4 soft `set_nl_cons` on the axle point, shrunk by 0.269. | Hard bounds on a measured state deadlocked Lab 2 task 4. |
| Q6 | `n_horizon` measured at the desk: N in {20, 40, 60}, largest with p95 solve < 50 ms. One default, in the node. | Constraints scale with discs x obstacles x N. Lab 2 had 100 in the node and 20 in the launch file. |
| Q7 | Two gates before new code: (1) copied node reproduces Lab 2 in Gazebo, (2) plank model loads and drives. | Each later failure has one suspect. |
| Q8 | `SCENARIOS` table plus a separate `footprint:=one\|three` parameter. | Footprint is the independent variable. |
| Q9 | Scenario 1: circle r 0.8 with obstacle (0, 0.62, 0.12), then a figure-8 with an obstacle by the crossing. | With no obstacle the footprint is in no constraint and the pair shows nothing. |
| Q10 | Scenario 2: corridor walls plus two posts r 0.10, gap 0.60 / 0.50 / 0.40 / 0.34 / 0.30 / 0.28. "Pass" = goal reached and true clearance > 0. | Free posts let the robot drive around them, so reach would leak into the result. |
| Q11 | Claude writes the code. Dominic studies `STUDY/Project Theory Guide.html` now and a full study guide after, and runs every ROS command. | Time after Lab 4. |
| Q12 | `project_sim.launch.py` starts Gazebo, the world, the plank Burger and the bridges: the sim's equivalent of `turtlebot3_bringup`. The MPC node runs in its own terminal. | Same two-step habit as on the robot. |
| Q13 | One `SCENARIOS` dict in `mpc_node.py`, and the launch file imports it. `scenario:=s2_030` is typed in both terminals. | One copy of the numbers. Lab 2's plot tool bug was two copies. |
| Q14 | Plan as if solo. Abdalrahman owns evaluation and report on paper. | See the conversation of 15 Oct. |

### Facts found while planning

- `odom` pose = `base_footprint` = wheel axle centre. The Burger chassis box is centred at x = -0.032 and its rear corners are 0.124 m from the axle. Lab 2's 0.105 disc under-inflated the body by about 19 mm.
- Lidar top is at about 0.19 m, so the plank goes above 0.20 m.
- do-mpc soft constraints add slack linearly (`+ penalty * eps`), which is an exact penalty: slack is zero whenever a feasible plan exists and the penalty beats the multiplier.
- There is no `lab2/LOG.md`. Lab 2's log is `lab2/report_notes.md` plus findings 7a-7e in `lab2/PLAN.md`.

### What was built

| File | From | Changed |
|---|---|---|
| `ros2_ws/src/r7021e_project/r7021e_project/mpc_node.py` | `lab2/.../mpc_controller.py` | docstring package name only |
| `ros2_ws/src/r7021e_project/rviz/project.rviz` | `lab2/.../rviz/lab2.rviz` | none |
| `ros2_ws/src/r7021e_project/package.xml`, `setup.cfg` | Lab 2 | package name |
| `ros2_ws/src/r7021e_project/setup.py` | new | installs rviz/, launch/, worlds/ by glob |
| `desk/sim_scenario.py` | `lab2/plots/sim_task.py` | node path |
| `analysis/plot_run.py` | `lab2/plots/plot_rosbag.py` | node path |

### Gate 0 (desk, no ROS): passed

`python3 desk/sim_scenario.py --task 4 --laps 1` on the copied node:

| | Lab 2 report notes | copy |
|---|---|---|
| obstacle closest | 0.2249 m | 0.2249 m |
| mean error after drive-on | 0.0025 m | 0.0026 m |
| max error after drive-on | 0.0463 m | 0.0463 m |
| solver failures | 0 | 0 |

The copy behaves like Lab 2 (one lap here, two laps in the notes). Next is gate 1 in Gazebo, run by Dominic: see `RUNSHEET.md`.

---

## 2026-10-15 (evening): gate 1 results, the project node, scenarios 1 and 2

Dominic ran gate 1 and then handed over for the rest of the day. Claude ran Gazebo headless on
`ROS_DOMAIN_ID=99` with its own `GZ_PARTITION`, so nothing touched Dominic's terminals.

### Gate 1 (Dominic's bags)

| bag | result | why |
|---|---|---|
| `gate1_t4` | **pass**. About two laps (10.13 m), closest approach 0.2195 m to the 0.225 keep-out (5.5 mm of soft slack), mean circle error 4.4 mm | The copy behaves like Lab 2. |
| `gate1_t3` | fail: robot never moved | Every `/cmd_vel` was exactly zero and `/mpc_prediction` had 0 messages, so the node never had a goal. The robot sat at (0.8, 0), where t4 left it, until Gazebo restarted at 83 s. |
| `gate1_t3-version2` | fail: same | Sat at the origin for 163 s, all-zero commands. |

`/new_position` is missing from both t3 bags, so the recorder never saw a goal publisher. The goal was
published either before the node was up (`ros2 topic pub` messages are not stored, so a late
subscriber never sees them) or from a terminal on a different `ROS_DOMAIN_ID` (trap 1).
**Reproduced:** with the goal published 10 s *after* the node started, the same node and
command drove to (1.78, 0.01). Task 3 also solves at the desk with N=20 (0 failures, 8.3 s).
So it was the order of operations, not the code. Project scenarios with a point goal now carry
the goal in `SCENARIOS`, so no project run depends on publish timing.

### RViz "showed nothing of the robot"

The Lab 2 RViz config has no RobotModel display and nothing that draws the body. It only has TF
axes, Odometry arrows, scan, obstacle markers and the predicted path. Fixed in
`rviz/project.rviz`: a RobotModel display (from `/robot_description`), plus a MarkerArray on
the new `/mpc_footprint` topic, where the node draws the plank and the discs it believes in at
the measured pose. Verified headless (RViz window captured): with the new config RViz shows the
Burger, plank, discs, posts and walls, both with and without `use_sim_time`, so sim time was
**not** the cause. The likeliest cause is the missing body display. A terminal on another
`ROS_DOMAIN_ID` (trap 1) would also show an empty RViz, and the bags can't tell the two apart.
`use_sim_time:=true` stays in the RUNSHEET because it's correct in sim, not because it fixes this.

### Decisions made without Dominic (blocker rule: course tools only, proposal goals only)

| # | Decision | Why |
|---|---|---|
| D1 | **N = 20 stays** (reach 0.44 m). Q6's rule applied on a quiet machine: N=20 p95 59 ms (s2_030 three), N=40 p95 174 ms, N=60 p95 367 ms. | Only N=20 keeps the p95 inside the 100 ms tick. The first sweep read 100+ ms even for Lab 2's own task, because Dominic's Gazebo GUI was still open: **the GUI roughly triples solve time.** Timed runs use `gui:=false`. |
| D2 | s1b obstacle moved from (0, 0.25) to **(0, 0.36), r 0.08**. | At (0, 0.25) the robot's start (the origin) was inside the one-disc keep-out (0.349 m). At 0.36 both start outside, three discs need no detour, and one disc needs about 9 cm. |
| D3 | Box is `(x_min, x_max, y_min, y_max)`, not a single ±B. | The scenario 2 corridor is long in x and narrow in y. A square box can't describe it. Same four soft constraints as Q5. |
| D4 | Footprint `burger` (one disc, 0.105) kept, beside `one` and `three`. | So the Lab 2 tasks can still be rerun as gates with Lab 2's own body. |
| D5 | Circle runs are **one lap** (Lab 2 used two). | Halves run time. One lap passes the obstacle once, which is all the pair needs. |
| D6 | `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)`. | **Bug inherited from Lab 2:** on Ctrl-C, Jazzy shut rclpy down before `finally: node.stop()`, the zero command failed with "publisher's context is invalid", and the robot kept its last command. Seen in the first headless run. Now Ctrl-C arrives as a plain KeyboardInterrupt, `stop()` publishes, and then rclpy shuts down. |
| D7 | Ground truth from Gazebo's PosePublisher plugin on the robot model, bridged to `/ground_truth` (PoseStamped). | One plugin and one bridge argument. The controller never subscribes to it. |
| D8 | Desk folder kept out of git: `project/.gitignore` → `desk/`, like `lab4/.gitignore`. The unpushed skeleton commit was rebuilt without `desk/sim_scenario.py`. | Dominic's instruction. |

### What was built today

- `mpc_node.py`: `FOOTPRINTS` (burger / one / three, radii from `disc_chain(n)`), `SCENARIOS`
  (lab2_t3, lab2_t4, s1a, s1b, s2_060 ... s2_028), soft box, a keep-out per obstacle per disc,
  `/mpc_footprint` markers, `/mpc_solve_time`, `rectangle_clearance()` for grading, figure-8
  reference, stop on shutdown.
- `worlds/burger_plank.sdf` (course Burger plus plank link plus ground-truth plugin) and
  `burger_bare.sdf` (Burger plus ground truth only). Both pass `gz sdf -k`. The plank inertia
  first failed validation: rounding broke ixx + iyy ≥ izz.
- `launch/project_sim.launch.py`: the sim's bringup. It imports `SCENARIOS` and spawns the
  obstacles and walls from it.
- `analysis/plot_run.py`: grades bags from ground truth against the exact rectangle, and plots a
  run or a pair.
- `desk/sim_scenario.py`: local only, gitignored.

### Scenario runs, first pass (no safety margin), and what they showed

All 16 runs recorded headless, GUI off, graded from ground truth (`analysis/plot_run.py`). These
bags are kept in `bags/margin0/` as evidence.

- **Scenario 2 matched every prediction.** One disc rejects every gap ≤ 0.50 m and stops
  1.1–1.3 m short without touching anything. Three discs pass every gap down to 0.28 m with real
  clearance (gap − 0.20)/2.
- **s1a with three discs failed.** The real plank grazed the obstacle by 2.1 mm at t = 22.4 s.
  At that moment odometry said (0.356, 0.715) and ground truth said (0.337, 0.695), 2.8 cm apart.
  The three-disc plan rode its keep-out with about 4 mm of real clearance to spare (desk
  prediction +3.7 mm), so odometry error used up the margin. The contact made the wheels slip,
  odometry then drifted **1.03 m** from truth, and the robot tracked its circle in the wrong
  place for the rest of the lap (mean path error 0.37 m).
- s1a with one disc was fine: 7.9 cm of real clearance. The big disc's waste is also its safety
  margin.

### More decisions made without Dominic

| # | Decision | Why |
|---|---|---|
| D9 | **`safety_margin` = 0.02 m** added to every keep-out (`SAFETY_MARGIN`, also a parameter). All 16 runs re-recorded with it. | The s1a graze above. Course precedent: Lab 3's `inflation_radius` 0.120 against the robot's 0.105. Cost: three discs now need 2 × (0.130 + 0.02) = **0.30 m**, and one disc 0.579 m. Desk with margin: s1a three-disc clearance +18 mm, s2_030 passes, s2_028 rejected. So the narrowest gap three discs pass becomes exactly the proposal's 0.30 m. |
| D10 | Code review fixes (`code-review` skill, 6 findings, all fixed). (1) `stop()` resets `last_cmd`, so delay compensation doesn't push the pose with a command the robot is no longer running. (2) Lab 2's "detour bigger than reach" startup warning is back. (3) Topic goals are clipped to the shrunk box. (4) "Reached in" is timed from the first non-zero command, not from bag start (12.7 s became 8.9 s, same as the desk). (5) The smoke test uses the node's N. (6) The robot entity name is one constant in the launch file. | Before the bags that count were recorded, as CLAUDE.md requires. |

### Results with the 2 cm margin (the runs that count)

16 Gazebo runs, GUI off, graded by `analysis/plot_run.py` from ground truth against the exact
rectangle. Every node shut down cleanly (no "context is invalid"). Figures are in `plots/`, and
the full tables are in `STUDY/Project Study Guide.html` §13.

**Scenario 2: narrowest gap passed: one disc 0.60 m, three discs 0.30 m.**

| gap | one disc | three discs |
|---|---|---|
| 0.60 | pass, clearance 0.200 | pass, 0.200 |
| 0.50 | stopped 1.18 m short, safe (0.150) | pass, 0.150 |
| 0.40 | stopped, safe (0.100) | pass, 0.100 |
| 0.34 | stopped, safe (0.075) | pass, 0.070 |
| 0.30 | stopped, safe (0.061) | **pass, 0.050**, the proposal's gap |
| 0.28 | stopped, safe (0.053) | stopped, safe (0.037), the margin's price |

All passes reach the goal in 8.9–9.1 s from the first command, the same as the desk.

**Scenario 1: both footprints complete both paths with no contact.**

| | path error max, one | path error max, three | real clearance, one | real clearance, three |
|---|---|---|---|---|
| s1a circle | 0.207 m | 0.075 m (2.8× less) | 0.089 m | 0.011 m |
| s1b figure-8 | 0.302 m | 0.027 m (11× less) | 0.025 m | 0.043 m |

**Solve time:** median of the per-run medians, 22 ms with one disc and 33 ms with three (1.5×).
The worst p95 was 83 ms. Everything stays inside the 100 ms tick with the GUI off.

### Where this leaves the timeline

12–19 Oct goals done in simulation: plank in Gazebo, three-disc footprint, Lab 2 tasks rerun as
gates (t4 passed; t3's failure was the goal publish, reproduced and passed), scenarios 1 and 2
with pairs. Next, 20–23 Oct: scenario 3, the obstacle centre as a TVP. Still open from today:
Dominic should rerun at least one s2 pair and s1a himself from `RUNSHEET.md` before the oral.
