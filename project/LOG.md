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
