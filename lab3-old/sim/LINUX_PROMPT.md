# Prompt for the Claude on the Linux machine

Dominic pastes this to the Claude that runs on his Linux machine. That Claude has ROS 2 Jazzy,
Gazebo and this repo. It has no other context, so this file gives all of it.

---

## Your job

You test the finished lab 3 code in Gazebo with Dominic. You also help him get used to the
commands, so that he can run and debug the lab alone in the lab hall. The hall has no internet.
Nobody can answer questions there. Everything you build for him must work with the network off.

You write two files in `lab3/`:

1. `RUNSHEET.md`: the commands that worked, in the same style as `LAB2_RUNSHEET.md` in the
   repo root. Read that file first and copy its style.
2. A new entry at the end of `lab3/LOG.md`: what you did, what went wrong, what could be better.

## What exists

The code is in `lab3/ros2_ws/src/r7021e_exploration/`. It is the Canvas template with additions.
Claude wrote the additions at a desk from our lab 3 plan (`r7021e-lab3-plan.docx`) and the lab
PDF. Nothing ran in ROS or Gazebo before now. A desk simulation with ROS stubbed out gave 12 of 12 full explorations of
the small maze. That says the logic works. It says nothing about Gazebo.

Read `lab3/LOG.md` first. It lists every file, every value, every change from the plan and every
weak point that nobody has tested.

| Part | What it does |
|---|---|
| `navigation_node` | Picks a frontier, plans with RRT*, publishes `/path`, `/rrt_tree`, `/goal_marker`. Also plans to a point that the RViz "2D Goal Pose" tool sends on `/goal_pose` (Task 2 test with a known goal) |
| `path_follower_node` | Follows `/path`. Slows down on `/scan`. Stops at the end of the path |
| `task1_baseline` | Task 1. Sends a 0.5 m path, then the reverse, and repeats |
| `exploration.launch.py` | Starts SLAM, RViz and the nodes. `use_sim_time:=true` for Gazebo. `task1:=true` runs Task 1 |
| `lab3/sim/maze_world.launch.py` | Starts Gazebo, a maze and a Burger. Worlds: `lab3_maze_small`, `lab3_maze` |

## Rules

- Dominic wants direct answers. No filler. No em-dashes.
- Coach him. Before a command, tell him what you expect to see. After it, compare. For the debug
  drills in phase 5, he types the commands and you only read the output.
- Do not rewrite the code. Do not add features. The teacher did not like the size of the old
  code. Fix only a real defect that you can show. Put each fix in its own commit. Write it in the log.
- If a change is a design choice and not a defect, stop and ask Dominic.
- Git: the author is Dominic only. Never add `Co-Authored-By` or any Claude line to a commit or
  to a pull request text. After each commit, run `git log -1 --format=%B` and check this. Use
  the git identity that is already set. If it is empty, ask him.
- Keep md files and code comments plain: short sentences, active voice, no em-dashes.
- Do not put bags or large files in git. Save bags in `~/bags/`.
- Small commits. Push to `origin main` when a phase is done.

## Phase 0: ready for offline (20 min)

1. Find the repo. Run `git pull`. Run `git log --oneline -12`.
2. Check the packages and tools. Run `gz sim --version`. Run `ros2 pkg prefix <name>` for each of: `slam_toolbox`,
   `turtlebot3_gazebo`, `turtlebot3_bringup`, `ros_gz_sim`, `ros_gz_bridge`, `rviz2`, `rqt_graph`,
   `tf2_tools`. Run `python3 -c "import numpy"`. Run `echo $TURTLEBOT3_MODEL`. It must say `burger`.
3. If a package is missing, install it now. Dominic is online now and will not be in the hall.
   Write the exact `apt install` line in `RUNSHEET.md`.
4. Turn the network off (Wi-Fi off). Keep it off for all other phases. If something needs the
   network, write down what, and find a way that does not.

## Phase 1: build (10 min)

```bash
cd ~/R7021E-group6/lab3/ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 pkg executables r7021e_exploration
ros2 launch r7021e_exploration exploration.launch.py --show-args
```

Pass: the build ends with no error. Four executables show: `frontier_detector_node`,
`navigation_node`, `path_follower_node`, `task1_baseline`. The launch file lists
`use_sim_time` and `task1`.

## Phase 2: Gazebo alone (15 min)

Terminal 1:

```bash
ros2 launch ~/R7021E-group6/lab3/sim/maze_world.launch.py world:=lab3_maze_small
```

Terminal 2. Dominic runs these. Ask him what each one should show before he runs it.

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /clock --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic info /cmd_vel -v
```

Pass: a Burger stands in the maze. `/scan` has a steady rate. `/clock` changes. The `odom` to
`base_link` transform prints. Look at the message type on `/cmd_vel`. The follower sends
`TwistStamped`. If the bridge wants `Twist`, the robot will not move. Show Dominic where to read
the bridge type, and then stop and ask him before you change anything.

Stop Gazebo with Ctrl-C. Run `ps aux | grep -E "[g]z sim"`. Kill what is left. Write this check in
`RUNSHEET.md`.

Two more worlds exist for Dominic's rehearsal of an unknown maze: `lab3_maze_blind_a` and
`lab3_maze_blind_b`. Do not look at their layout and do not describe it to him. Start each one once
with `gui:=false`. Pass if `/scan` has a steady rate and `ros2 topic echo /clock --once` works.
Then stop it. Do not run exploration in them. He does that himself.

## Phase 3: Task 1 (15 min)

Terminal 1: Gazebo as above. Terminal 2:

```bash
ros2 launch r7021e_exploration exploration.launch.py use_sim_time:=true task1:=true
```

Pass: RViz opens and shows the map. These three commands print `True`:
`ros2 param get /path_follower_node use_sim_time`, `ros2 param get /slam_toolbox use_sim_time`,
`ros2 param get /task1_baseline use_sim_time`. The robot drives 0.5 m forward, stops, drives back,
and repeats. The launch log prints `leg 1`, `leg 2`, and so on. Check `ros2 node list`. There must be
no `navigation_node` in it. Look at `ros2 topic echo /path --once` and `ros2 topic echo /cmd_vel`.

If the log repeats `TF lookup ... failed`, check these in order: `ros2 lifecycle get /slam_toolbox`,
`ros2 param get /slam_toolbox use_sim_time`, `ros2 run tf2_tools view_frames`.

## Phase 4: full exploration (30 min)

Run the small maze first. Then the big maze. Make a folder for each run:
`lab3/results/<date>-<world>-<n>/`.

```bash
ros2 launch r7021e_exploration exploration.launch.py use_sim_time:=true 2>&1 | tee ~/R7021E-group6/lab3/results/<run>/launch.log
ros2 bag record -o ~/bags/<run> /map /frontiers /path /cmd_vel /odom /tf /tf_static /scan /clock
```

Start the recorder before the launch. Stop the launch first, then the recorder.

Look at these in RViz: the map, the green path, the RRT tree, the red goal sphere.

The navigation node prints one line for each plan:
`t=... goal=(x, y) H=... L=... I=... turn=... candidates=... known=...m2 plan_time=...s`.
It also prints `arrived`, `stalled`, `path is old`, `not reachable` and `exploration finished`.

For each run, write `summary.md` in the run folder:

- Did it print `exploration finished`? At what `t`?
- Number of plan lines (goals), number of `stalled`, number of `not reachable`.
- `known` area at the end. Time when `known` first passed 90 % of the final value.
- Did the robot touch a wall? Did it get stuck? Did the map jump or double? If the map jumps,
  set `do_loop_closing: false` in `config/slam_config.yaml`, run again, and write the change in
  the log. Loop closure stays on if the map does not jump.
- The longest `plan_time`.

Commit the `launch.log` and `summary.md` of each run. Do not commit the bag.

Do not tune any value yet. A later session will run experiments on these logs. Change a value
only to remove a defect, and write why.

## Phase 4b: one proof run for each task (45 min)

The lab has five tasks. Dominic must show each one in simulation. Task 1 is phase 3 and Task 5
is phase 4. These runs are for Tasks 2, 3 and 4. Use `lab3_maze_small`, one run each, and label
every number `[gazebo]`. They are single runs, so they show behaviour and are not report data.

Task 2, known goal. Start a normal full launch. When the first plan line prints, click the RViz
"2D Goal Pose" tool on a free spot of the map that the robot has already seen. Expected: the log
prints `goal from RViz: (x, y)`, the next plan line has that goal, the green path and the RRT tree
go to it, and the robot drives there. Then click inside a wall. Expected: `not reachable, crossed
off`, and no crash. Check the message with `ros2 topic echo /goal_pose --once`. If the click does
nothing, check that the RViz Fixed Frame is `map`. Exploring goes on after the robot arrives.

Task 3, collision avoidance. Run once with `inflation_radius` 0.0 in `DEFAULTS`, then once with
the normal 0.105. Write down for each run: did the robot touch a wall, how many `stalled` lines,
how close did the green path run to the wall in RViz.

Task 4, exploration gain. Run with `info_weight` 0.0 (nearest goal only), 0.10 (normal) and
0.30. Write down for each run: time of `exploration finished`, time when `known` first passed
90 % of its final value, number of goals.

Set `DEFAULTS` back after each run (`git diff` must be empty at the end, and `--symlink-install`
means no rebuild). Put the three tables in `lab3/results/<date>-task-proofs/summary.md`. Say in
the file that one run per setting cannot separate a real effect from RRT* randomness. If a setting
changes the result a lot, run it a second time before you write the claim.

## Phase 5: debug drills (40 min)

Dominic does each drill. You ask what he expects, watch, and help only if he asks. For each
drill, add one row to the "when it breaks" table in `RUNSHEET.md`: symptom, command that shows
the cause, fix.

| Drill | Break it by | What he should find |
|---|---|---|
| 1 | Start Gazebo, then open a terminal and do not `source` the workspace | `ros2 launch` says the package is not found |
| 2 | Run Task 1 with `use_sim_time` left out | TF and time warnings. Name the command that shows it |
| 3 | `export ROS_DOMAIN_ID=5` in one terminal only | `ros2 topic list` shows nothing. Fix it |
| 4 | Kill `path_follower_node` while the robot drives | The robot keeps its last speed or stops. Find which. Then use the stop command |
| 5 | Start the launch twice | Two nodes with one name. Find it with `ros2 node list` |
| 6 | Break nothing. Run `ros2 topic hz /scan` and `ros2 topic hz /map` | Learn the normal rates. Write them in the runsheet |
| 7 | Run `ros2 run rqt_graph rqt_graph` | Name every node and topic between `/scan` and `/cmd_vel` |
| 8 | Change `info_weight` in `DEFAULTS` at the top of `navigation_node.py`. Restart the launch | The value changes with no rebuild, because of `--symlink-install`. Read it back with `ros2 param get /navigation_node info_weight`. Then try `ros2 param set` and see that it changes nothing: the node reads its values once at start |

The stop command must be in `RUNSHEET.md` at the top:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{}"
```

## What to leave in the repo

1. `lab3/RUNSHEET.md`: every command that you ran and that worked, in this order: stop command,
   terminal start lines, build, Gazebo, Task 1, full run, recording, real robot, when-it-breaks
   table, how to change a value, where to read a message type or a parameter offline.
   For the real robot, copy the steps from `LAB2_RUNSHEET.md`. Mark them "not tested on the robot".
2. `lab3/results/...`: the logs and summaries.
3. A new entry in `lab3/LOG.md`, dated, with these headings: what we did, what went wrong, what
   we would do better, what is still not tested.

At the end, give Dominic a list of the three things most likely to go wrong in the lab hall, and
the command that finds each one.

## Time

About 2 h 55 min in total. Dominic does phases 0 to 4 on Thursday evening and phases 4b and 5 on Friday evening. Stop at 4 hours of work in one day. If a phase takes much more than
planned, say so and ask Dominic whether to continue.
