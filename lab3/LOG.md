# Lab 3 log

Append only. Each entry has a date. Write what we did, what went wrong, and what we would do
better. We use this file when we write the report.

Labels on numbers: **[desk]** is the offline simulation on a laptop with no ROS. **[gazebo]** is a
Gazebo run. **[robot]** is a run on the TurtleBot3. Do not join numbers with different labels
in one plot.

---

## 2026-10-07: first version of the code, written at a desk

### How we worked

The group decided to finish the code first and study it afterwards. Claude (AI assistant) wrote
the additions to the Canvas template, following `r7021e-lab3-plan.docx` and the lab PDF. Claude
took tuned values and the node structure from `advanced-robotics-2026`, lab 3. Our teacher
found that code too big. We used the classmate repo `willmerw/R7021E-` only to judge how much
code a student version needs: about 570 lines in one navigation file. We did not copy it.

Nothing ran in ROS or Gazebo when this entry was written. Every check was done at a desk with
ROS stubbed out. The real tests are the next entries.

### What is in the package

| File | Change |
|---|---|
| `navigation_node.py` | New. 602 lines, one file. RRT*, frontier clusters, score H, goal handling |
| `path_follower_node.py` | Three additions: scan slow-down, stop at the path end, guarded TF lookup |
| `task1_baseline.py` | New. Task 1. Sends a short path, then the same path reversed, and repeats |
| `frontier_detector_node.py` | Not changed |
| `exploration.launch.py` | `use_sim_time` goes to every node, SLAM and RViz. New argument `task1:=true` |
| `slam_config.yaml` | Four values from the old tuning. Loop closure stays on |
| `exploration.rviz` | Two marker displays: RRT tree and goal |
| `lab3/sim/` | Two maze worlds and `maze_world.launch.py`. Not part of the package |

SLAM values changed from the template: `map_update_interval` 5.0 to 1.0, `minimum_time_interval`
0.5 to 0.1, `minimum_travel_distance` 0.5 to 0.2, `minimum_travel_heading` 0.5 to 0.3.

Loop closure is on, as in the plan. If the map jumps during a run, set `do_loop_closing: false`
in `config/slam_config.yaml` and write the change in this file.

### Values we use

| Name | Value |
|---|---|
| inflation radius | 0.105 m (becomes 3 cells, a 0.15 m collar) |
| RRT* iterations, step, goal bias | 1500, 0.30 m, 10 % |
| rewire radius, goal tolerance | 0.60 m, 0.15 m |
| iterations after the first solution | 200 |
| retry budget | 15000 iterations |
| score H | path length - w * I + 0.15 * turn, with w = 0.10 m per cell |
| information radius | 0.75 m |
| smallest cluster, candidates per cycle | 5 cells, 6 |
| arrival, stall, old path, stop | 0.20 m, 0.05 m in 8 s, 20 s, 10 empty cycles |
| follower scan check | stop at 0.18 m, full speed from 0.30 m, strip half width 0.10 m |

### Where the code is not the plan

We added these. Each one has a reason.

1. **Planning runs on a 1 s timer.** The template planned inside the map callback. Planning can
   take longer than the map period, and the arrive, stall and old-path rules need a steady clock.
2. **`frontier_topic` default is `frontiers`.** The template says `frontier`. The launch file has
   no remap, so the template subscription never receives a message.
3. **Single occupied cells are removed before inflation.** This is scan noise. The old repo found
   this mattered more than the radius.
4. **The inflation radius is rounded up to whole cells.** 0.105 m is 2.1 cells. We use 3.
5. **The tree starts at the nearest free cell** when the robot stands inside the collar. The robot
   position goes at the front of the path.
6. **Only free cells that connect to the robot count as free.** A flood fill finds them. Without
   this, 5 of 12 desk runs on the small maze stopped at 57 % coverage [desk]. The middle of a thin
   frontier sliver was in a part of the free space that did not connect to the robot, and RRT*
   could never reach it. With the flood fill, 12 of 12 runs reached 100 % [desk].
7. **Clusters are 8-connected.** The frontier detector uses 4 neighbours, and diagonal frontier
   cells would split into separate clusters.
8. **Waypoints are 0.10 m apart.** The follower pops waypoints inside 0.20 m, and a sparse path
   makes it cut corners.
9. **At most 6 candidates per cycle**, the nearest by straight line. The plan says "to each". The
   cap keeps the cycle short.
10. **Goals that RRT* cannot reach are crossed off**, like goals that we reached or that stalled.
    Without this, the node plans to them again every second.
11. **The termination count starts after the first goal.** Before the first map, there is
    nothing to explore and the node must not stop.
12. **The follower measures scan age at reception**, not from the message stamp. The robot and
    the laptop can have different clocks.
13. **The world files have a ground plane in the file.** The course generator used a Fuel
    download, which fails with no network. `maze_world.launch.py` finds the worlds next to
    itself, because the old launch file needed a package that is not in this repo.

### What we measured at the desk [desk]

The desk simulation runs our real navigation and follower code. ROS is stubbed. The truth map
comes from the world files. The lidar has 360 rays and a 3.5 m range. The pose is perfect. The
map update is exact ray casting. The robot follows `cmd_vel` with no delay at 0.15 m/s and slides
along walls. It does not model slam_toolbox, loop closure, odometry drift, Gazebo contacts or
ROS timing. It shows that the logic works. It does not predict Gazebo times.

- Small maze, 12 runs: all reached 100 % coverage. Time to 90 %: 93 to 160 s, median 98 s.
  Whole run: 123 to 181 s. Driven: 14 to 22 m. Wall contact: 1 to 11 steps of 0.1 s.
- Big maze, 4 runs: all reached 100 %. Time to 90 %: 386 to 428 s. Whole run: 472 to 508 s.
  Driven: 61 to 65 m.
- RRT* on 25 random pairs in the big maze, 3 to 25 m apart [desk]: 1500 iterations solved 6,
  6000 solved 21, 15000 solved all 25. Mean time per call 0.45 s, longest 1.9 s. Mean path
  length was 1.00 to 1.02 times the grid shortest path. This is why the retry budget is 15000.
- In one first-plan check on the small maze, the closest path point was 0.130 m from a true wall.
- Follower, one test per row: wall at 0.24 m gives half speed, wall at 0.18 m gives zero, a side
  wall at 0.15 m does not slow the robot, a scan older than 0.5 s gives zero, a lost TF gives no
  command, and the robot stops at the end of the path.

### What went wrong while writing it

- **First desk model stopped the robot on contact.** The robot cut a corner at a wall post and
  stayed there. The goal was crossed off after 8 s. A run ended at 88 % with goals missing. The
  follower aims at the first waypoint more than 0.2 m away, so it cuts a corner by up to
  about 0.03 m past the path. The margin between the collar and the robot radius is only about
  0.025 m. We changed the desk model so the robot slides along the wall, which is closer to
  Gazebo. Expect corner touches in Gazebo as well.
- **The plan rule for the goal point was not enough** (item 6 above). The plan says to take the
  middle of the cluster if it is free. We found that "free" must also mean "connected to the robot".
- **Launch argument order.** We first put `use_sim_time` and `task1` in the nodes at the top of
  the launch file and left their `DeclareLaunchArgument` lines at the bottom, as in the template.
  A default is only set when its declaration runs. A launch with no `use_sim_time:=...` on the
  command line, which is the real-robot case, would have failed. The two declarations are now first
  in the list. We could not run the launch file at the desk, so this is the first thing to check.
- **A test expectation was wrong.** The wall at 0.10 m gave full speed because the lidar
  ignores returns closer than its minimum range of 0.12 m. This is a real blind zone. The robot
  stops at 0.18 m, so it should not get there. Check it in Gazebo.

### Known weak points to check in Gazebo

- A robot in a closed pocket of the map stops and says "finished" after 10 empty cycles, even if
  frontiers remain somewhere that it cannot reach.
- Crossing off an unreachable goal is permanent. If the map later opens a way, only a goal more
  than 0.30 m away from the crossed-off point can use it.
- The follower treats the scan frame as `base_link`. The laser is 3 cm behind the centre.
- The first plan on a real map can take a few seconds. The robot keeps following the old path.

### What we would do better

- Write the log while working, not after. This entry was written in one pass.
- Run the first Gazebo test with the small maze and a fixed order: Task 1, then one planning
  cycle, then the full run.
- Keep the report numbers apart by label. A desk number must not go into a result plot.

### Next entry

Pull on the Linux machine, build, run Task 1 in the small maze, then the full run in both mazes.
Write the commands that worked in `RUNSHEET.md` and the results in this file.

---

## 2026-10-08: goal from RViz for Task 2

### What we did

We compared the lab PDF with the code. Task 2 says: "Test in the maze for relatively simply path
generation (with a known goal)." The node chose its own goals and had no way to take one from the
user. We added one subscription to `/goal_pose`. The RViz tool "2D Goal Pose" publishes on that
topic. The node plans to the clicked point at its next tick, with the same RRT* as for a frontier
goal. After the robot arrives, the node goes back to frontier exploring.

The change is 11 lines in `navigation_node.py`: a subscription, a callback and one line in
`plan_path`. The RViz file already had the tool and the topic.

### What we checked [desk]

A click in free space gives a path that ends at the clicked point and stays inside the planning
mask. A click inside a wall gives `not reachable, crossed off` and no crash. Three desk runs of
the small maze reach 100 % as before.

### What is still not tested

The click in RViz and Gazebo. Phase 4b of `lab3/sim/LINUX_PROMPT.md` tests it, and it also holds
one proof run each for Tasks 3 and 4.

### Where it differs from the plan

This is a change of scope, so we write it down. The plan holds no known-goal mode. The lab text
asks for a test with a known goal, and the planner test in the plan was a hard-coded goal.

### 2026-10-08, later: study files and one more deviation

We wrote four study files in `lab3/STUDY/`: `THEORY.md`, `QUESTIONS.md`, `CODEMAP.md` and
`COMMANDS.md`. Writing the theory sheet showed a deviation that the first entry did not list.

14. **The information term counts frontier cells, not unknown cells.** The plan and the lab text
    say to count unknown cells in a reduced range. `score` counts frontier cells within 0.75 m of
    the goal. This is a proxy, and it is the count that the old repo tuned (w = 0.10 m per cell).
    A switch to unknown cells would change the scale of I by a large factor, so w would need
    tuning again. We keep the proxy for now. The decision is open. Dominic defends it at the oral
    as a proxy, or asks for the switch.

Also found: `navigation_node` has the node name `path_planner_node` in the code, and the launch
file renames it to `navigation_node`. The follower does the same (`path_follower` and
`path_follower_node`). Use the launch names in `ros2 param get`.

### 2026-10-08, later still: two worlds for an unknown maze

We added `lab3_maze_blind_a` (7 by 7 cells, 5.6 m) and `lab3_maze_blind_b` (9 by 9 cells, 7.2 m).
A small script made them in the style of the course generator, with random depth-first mazes,
a few extra openings and an open start cell. Dominic does not look at the layout. He starts them
with `gui:=false` and learns the maze from the RViz map, as he will in the hall.

Desk check [desk], two runs each: both mazes reach 100 %. Blind A: 245 to 249 s, 31 to 32 m
driven. Blind B: about 450 s, 60 m driven. They do not load in Gazebo yet. Phase 2 of the Linux
prompt checks this.

### 2026-10-08, later still: the oral file

The oral on Monday 12 Oct has a talk and a demo. We added `lab3/STUDY/ORAL.md`. It holds five
headings for the opening, the demo order, three demo layers with a 2-minute fail-over rule, the RViz
colours with what to say, and a failure script. It holds no sentences for the talk. Dominic writes
those himself on Sunday evening.

Not known yet: the time of the oral, whether the real robot is available, and whether slides are
needed. The RViz map colours are not read from the `.rviz` file (`Color Scheme: map` is the RViz
default). Check them in the first Gazebo run and fix the table in `ORAL.md`.

## 2026-10-10: the planner publishes its inflated map

### What we did

- `navigation_node` publishes `/inflated_map` (`OccupancyGrid`) each time it plans. A cell is 100
  where the map is known and the robot centre may not go: a wall or the padding around it. All
  other cells are 0. RViz shows it as "Inflated map", colour scheme costmap, alpha 0.5.
- Why: Task 3 needs proof of how close the path runs to a wall. The path, the padding and the
  wall are now in one view, live and in the bag.
- Idea from another team's lab 3, which also publishes an inflated map. We wrote our own from
  the mask the planner already uses. No code was copied.

### What we checked [desk]

- A fake 20 x 10 map with a wall 2 cells thick gives 8 blocked cells in a row: the wall and 3
  cells of padding on each side. Unknown cells stay 0.

### What is still not tested

- How the costmap colours look over the map in RViz. If the layer hides the map, lower its alpha.
- Add `/inflated_map`, `/rrt_tree` and `/goal_marker` to the bag record line. Without them the
  replay video cannot show them.

## 2026-10-10, evening: first Gazebo runs, fixes, and a check for the real robot

Claude ran the simulations, at Dominic's request. Dominic has not yet typed these commands
himself. All numbers are [gazebo] unless marked. Table of all runs:
`results/2026-10-10-overview.md`.

### What we did

- Phases 0 to 4. Packages present (`turtlebot3_gazebo` comes from `~/turtlebot3_ws`, sourced
  by `~/.bashrc`). Build passes. Gazebo, both blind worlds (scan and clock only) and Task 1 pass.
- 20 full exploration runs in the small, big and narrow-door worlds. Each bag also held the
  true Gazebo pose, so wall contact and SLAM error are measured against the truth.
- Seven fixes, each in its own commit, each from a run that failed:

| Commit | Fix | The run that showed it |
|---|---|---|
| 3c8c19c | Follower backs off 0.10 m when blocked ahead for 2 s | small-1: froze at a corner, goal crossed off, 5.5 of 16 m2 |
| e6262dd | Stalled goals get 2 more chances before `exploration finished` | small-1, big-1 |
| 7418286 | `look_ahead` 0.20 to 0.12 m | small-2: aimed past a corner and drove into it, for 4 minutes |
| 9944cb1 | `kp_yaw` 2.0 to 1.0, `max_w` 1.0 to 0.6 rad/s | Gazebo odometry turned 41 % more than the truth, worst when turning while driving |
| cf4fab1 | SLAM loop closure off | small-4 and small-5: pose error jumped from 0.15 to 1.7 m in 4 s |
| aa65d6f | The bottom 0.02 m of the slow-down ramp counts as blocked | small-7, big-2: crept at 0.00 to 0.01 m/s and never backed off |
| f632b52 | Follower sends zero on Ctrl-C and when the pose is lost | Ctrl-C left the robot driving at 0.15 m/s |

- Also: the planner publishes `/inflated_map` (210b3af), a narrow-door test world, a video
  script (`video/bag_to_video.py`), and `RUNSHEET.md`.
- Checks for the real robot, without the robot:
  - `turtlebot3_node` has a heartbeat to the motor board but no `cmd_vel` timeout (read from
    the installed binary). The robot keeps its last command.
  - The Lab 1 robot bags: the robot clock was 325 days behind the laptop. TF is
    `odom -> base_footprint -> base_link -> base_scan`. The laser gives 0.0, not inf, for no
    return, on 41 % of rays. `/cmd_vel` on the robot is TwistStamped.
  - All 635 Lab 1 robot scans gave the same forward clearance in the follower as an
    independent check. No 0.0 range was used.
  - A Lab 1 robot bag played into the full stack with `use_sim_time` false: SLAM built a map,
    the planner planned, the follower published. The 325 day clock offset did not stop it.
  - Laptop receive gaps in the Lab 1 bags: up to 3.6 s, 30 gaps over 0.5 s in one run, while
    the message stamps stayed 0.15 to 1.0 s apart. The data was late, not lost.

### What went wrong

- The desk version explored 34 % of the small maze. The desk simulation had perfect wheels
  and no corners that a pure pursuit could cut, so it could not show the freeze at a corner.
- Two of our own tries made things worse and were taken back. Lower SLAM travel thresholds
  (small-4) broke SLAM. The trail-as-free fix (small-10, narrow-trail) made paths hug walls.
  It was never committed.
- One good run (small-6) hid a bug that the next run with the same code (small-7) showed.
  One run is not evidence.
- Claude's test scripts first killed their own shell (pkill matched its own command line) and
  first left bags without `metadata.yaml`. A background job from a script ignores SIGINT, so the
  recorder never stopped. SIGTERM works. In a terminal, Ctrl-C works as usual.
- Gazebo keeps two `gz sim` processes after Ctrl-C, every time.
- The spawn cell in `lab3_maze_small` has a wall 0.35 m ahead. Task 1 needs `x_pose:=-0.8`.
- `ros2 topic hz /scan` printed nothing on this laptop.

### What we would do better

- Run two runs for every change from the start, and measure against the Gazebo truth from the
  first run. Odometry said the robot drove through walls, and that was odometry drift.
- Test the stop on shutdown first. It is the most dangerous defect, and it is the cheapest
  to test.

### What is still not tested

- Nothing has run on the robot. Real odometry slips less than Gazebo. The values `kp_yaw`,
  `max_w` and loop closure off come from Gazebo.
- Passages from 0.30 to 0.50 m. The robot can enter a 0.40 m door and then the padding closes
  it behind the robot, or the follower blocks at the door post. See the runsheet rule.
- The frontier behind a corner next to an arrived goal falls inside the 0.30 m retire radius
  and is skipped (small-10). Seen once.
- Lab 1 bag gaps of up to 3.6 s: the robot drives on a stale command for that long. Only
  `max_v` lower helps.
- RViz colours of the inflated map. RViz replay of a bag with the robot clock offset. The video
  script does not depend on either.
- Tasks 2, 3 and 4 proof runs, and the phase 5 drills. Dominic does these.

### Late runs, same evening

- `inflation_radius` 0.16 (4 cells): small maze 5.4 m2, 3 stalls, 22 back-offs at a corridor
  mouth. Stopped after one run. Not adopted.
- Follower scan check off (`stop_distance` 0.0, `slow_distance` 0.01), not committed: 2 of 2
  small runs full (15.1 and 15.0 m2), 0 back-offs, closest 0.107 m. One big run still running.
  Decision for Dominic: see `STUDY/WHY.md`.
- Big maze with the scan check off: SLAM error 2.62 m, and the robot centre came 0.038 m from
  a wall (contact). The scan check stays on. Kept as committed.
