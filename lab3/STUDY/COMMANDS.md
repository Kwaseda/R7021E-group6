# Lab 3 commands

Written at a desk on 2026-10-08. No command here has run yet. Phase 1 to 4b of
`lab3/sim/LINUX_PROMPT.md` runs them in Gazebo. Then `lab3/RUNSHEET.md` exists. When the two
files disagree, `RUNSHEET.md` wins. Fix this file to match.

Rows marked "robot: not tested" have not run on a TurtleBot.

Robot number X: turtle X, IP `192.168.50.X0`, domain `3X`. Example: turtle 5 is 192.168.50.50, domain 35.

---

## Stop the robot

Keep this in a spare terminal.

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{}"
```

## Every terminal starts with these lines

```bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=3X
source ~/R7021E-group6/lab3/ros2_ws/install/setup.bash
```

- No `export`: the terminal sees nothing and gives no error. Check with `printenv ROS_DOMAIN_ID`.
- No `source`: you get "Package 'r7021e_exploration' not found".
- In Gazebo at home, any domain works. Use the same number in every terminal.
- The lab PDF puts these lines in `~/.bashrc` and uses `~/ros2_ws`. We use the repo workspace.
  Your `~/.bashrc` can hold the first two lines.

## Build

```bash
cd ~/R7021E-group6/lab3/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

After this, edits to a `.py` file need no rebuild. Restart the launch. A change to `setup.py` or
`package.xml` needs a rebuild.

---

## Simulation

### Start Gazebo (terminal 1)

```bash
ros2 launch ~/R7021E-group6/lab3/sim/maze_world.launch.py world:=lab3_maze_small
```

| Argument | Values | Default |
|---|---|---|
| `world` | `lab3_maze_small` (4 m by 4 m), `lab3_maze` (7.2 m by 7.2 m) | `lab3_maze_small` |
| `gui` | `true`, `false` | `true` |
| `x_pose`, `y_pose` | spawn position in metres | 0.0, 0.0 |

Run it by path, as shown. It is not part of the ROS package. After Ctrl-C, look for leftovers:

```bash
ps aux | grep -E "[g]z sim"
```

### Start the stack (terminal 2)

```bash
ros2 launch r7021e_exploration exploration.launch.py use_sim_time:=true
```

| Argument | Values | Default | Meaning |
|---|---|---|---|
| `use_sim_time` | `true`, `false` | `false` | `true` in Gazebo. Leave it out on the robot |
| `task1` | `true`, `false` | `false` | `true` runs `task1_baseline` instead of `navigation_node` |
| `slam_params_file` | a path | `config/slam_config.yaml` | another SLAM setting file |
| `autostart` | `true`, `false` | `true` | start slam_toolbox by itself |

See all of them: `ros2 launch r7021e_exploration exploration.launch.py --show-args`.

`ros2 launch` takes `name:=value`. It rejects `--ros-args`. `ros2 run` takes `--ros-args -p
name:=value` and ignores `name:=value`. Mixing them is easy under stress.

### The five tasks

| Task | Command | You should see | Proof to keep |
|---|---|---|---|
| 1. Baseline | `... exploration.launch.py use_sim_time:=true task1:=true` | The robot drives 0.5 m ahead, stops, drives back, repeats. Log: `leg 1`, `leg 2` | `ros2 node list` has no `navigation_node`. `ros2 topic echo /path --once` shows about 6 points in frame `map`. A short screen recording |
| 2. Planner, known goal | `... exploration.launch.py use_sim_time:=true`, then click in RViz | After the click: `goal from RViz: (x, y)`, a plan line with that goal, a green path, the RRT tree, the robot drives there | The log lines, an RViz screenshot of the tree and path |
| 3. Collision avoidance | Same launch. Run twice: `inflation_radius` 0.0, then 0.105 | With 0.0 the path runs along the walls and the robot touches them. With 0.105 it keeps 0.15 m away | Two RViz screenshots. Number of `stalled` lines for each run |
| 4. Exploration gain | Same launch. Run with `info_weight` 0.0, 0.10, 0.30 | Different goal order. The log shows H, L, I and turn for each goal | A table: time to 90 %, total time, goals, for each weight |
| 5. Full system | `... exploration.launch.py use_sim_time:=true` in `lab3_maze` | The robot explores alone. `exploration finished` at the end | Bag, RViz screen recording, the launch log |

Task 2 click, in detail. In RViz press the "2D Goal Pose" button in the top bar. Click a free
spot on the map that the robot has already seen. Drag a short way to set a direction. The
direction is ignored. Click on a wall to see `not reachable, crossed off`. The Fixed Frame in
RViz must be `map`. After the robot arrives, the exploring goes on.

Task 3 and 4 change a value. Edit the number in `DEFAULTS` at the top of
`navigation_node.py`, save, and restart `exploration.launch.py`. Do not rebuild. Put the value
back afterwards. `git diff` must be empty.

### What the planner prints

One line for each plan. This example has made-up numbers:

```
t=84.2 goal=(1.25, -0.80) H=2.31 L=2.05 I=18 turn=0.40 candidates=4 known=9.4m2 plan_time=0.31s
```

| Field | Meaning |
|---|---|
| `t` | simulation time in seconds |
| `goal` | the goal the node chose |
| `H`, `L`, `I`, `turn` | the score and its parts (T4) |
| `candidates` | how many goals it compared |
| `known` | area of the known map |
| `plan_time` | real seconds for the whole cycle |

Other lines: `arrived at`, `stalled, crossing off the goal`, `path is old, planning again`,
`goal (...) not reachable, crossed off`, `exploration finished: N goals, X s since first plan`,
`goal from RViz`.

### Record a run

Start the recorder first. Stop the launch first, then the recorder. Press Ctrl-C once and wait.

```bash
mkdir -p ~/bags && cd ~/bags
ros2 bag record -o run1 /map /frontiers /path /cmd_vel /odom /tf /tf_static /scan /clock
ros2 bag info run1
```

Also keep the launch log:

```bash
ros2 launch r7021e_exploration exploration.launch.py use_sim_time:=true 2>&1 | tee ~/run1.log
```

The lab asks for a video of the robot and a screen recording of RViz. The GNOME recorder
(Ctrl+Alt+Shift+R) can stop after 30 s. Check on your laptop, before the hall:

```bash
gsettings get org.gnome.settings-daemon.plugins.media-keys max-screencast-length
gsettings set org.gnome.settings-daemon.plugins.media-keys max-screencast-length 0
```

---

## Read things with no internet

```bash
ros2 interface show nav_msgs/msg/OccupancyGrid
ros2 interface show geometry_msgs/msg/TwistStamped
ros2 topic info /cmd_vel -v                  # message type and QoS on both ends
ros2 topic hz /scan                          # rate. Same for /map
ros2 node list
ros2 param list /navigation_node
ros2 param get /navigation_node info_weight
ros2 pkg executables r7021e_exploration
ros2 run tf2_tools view_frames               # writes frames.pdf
ros2 run rqt_graph rqt_graph                 # nodes and topics as a picture
ros2 lifecycle get /slam_toolbox             # must say active
ros2 topic pub --help                        # all options of pub
```

Read the values in `DEFAULTS`. `ros2 param set` does not change the planner, because the node
reads its values once at start.

---

## A maze that is built and unknown (robot: not tested)

### Before the hall, with internet

1. The workspace builds. `ros2 pkg executables r7021e_exploration` lists four executables.
2. `lab3/RUNSHEET.md` is on your laptop and on paper. Print or take a photo.
3. You have run Tasks 1 to 5 in both Gazebo mazes with the network off.
4. `~/bags` exists. A screen recorder works and does not stop at 30 s.
5. The laptop is charged. The robot battery is charged.
6. You know the answers in `QUESTIONS.md` for T1 to T6.

### In the hall

1. Connect to the router Wi-Fi. The TA helps. The password is on the router.
2. Place the robot in the maze. Choose a start where the nearest wall is more than 0.3 m away,
   with open floor 0.5 m ahead and behind. The map origin is where the robot starts.
3. Terminal A. Open a shell on the robot. The password is `turtle`.

   ```bash
   ssh turtle@192.168.50.X0
   ros2 launch turtlebot3_bringup robot.launch.py
   ```

   Keep this running. Do not export a domain in this terminal.
4. Terminal B, on your laptop. Run the three start lines. Then check:

   ```bash
   ros2 topic list                    # /scan, /odom, /tf, /cmd_vel are there
   ros2 topic hz /scan                # about 5 Hz
   ros2 topic info /cmd_vel -v        # TwistStamped, one subscriber
   ```

5. Compare the clocks. Run `date +%T` on the laptop and on the robot. They must differ by less
   than 1 s. A big difference gives TF errors that look like a software fault.
6. Task 1 first. It needs 0.5 m of free floor ahead of the robot.

   ```bash
   ros2 launch r7021e_exploration exploration.launch.py task1:=true
   ```

   There is no `use_sim_time`. The robot drives forward, stops, drives back. Stop the launch.
7. Full run:

   ```bash
   ros2 launch r7021e_exploration exploration.launch.py
   ```

   Keep your hand on the spare terminal with the stop command. Keep a hand near the robot.
8. Start the bag recorder and the screen recording before the launch. Start a phone video of the robot.
9. When the run ends or you stop it, take an RViz screenshot of the whole map.

If the TA allows teleop to check SLAM first, run `ros2 run turtlebot3_teleop teleop_keyboard` in
an SSH shell on the robot. Stop teleop before you start the exploration. Both write to `/cmd_vel`.

### What is different from the sim mazes

| In Gazebo | In the unknown maze |
|---|---|
| Start is (0, 0) in a free cell | Start is where you place the robot. Check it by eye |
| The world is known, so coverage can be computed | Nothing says how much remains. Watch `known=` in the log. When it stops growing, the map is done |
| Time is simulated, `use_sim_time:=true` | Real time. Leave `use_sim_time` out |
| Perfect wheels, clean laser | Wheel slip, noisy laser at about 5 Hz, odometry drift. The map can bend |
| Corridors are about 0.8 m wide | Measure them with a ruler. The planner needs more than about 0.35 m between wall faces |
| No battery | The battery and the lab slot limit the run time. Decide a stop rule before you start |

Narrow corridors: if goals in a narrow part get `not reachable, crossed off`, the collar closes
the corridor. A smaller `inflation_radius` of 0.075 m (2 cells) opens it. The robot then has a
0.10 m collar instead of 0.15 m, and the laser slow-down is the only margin left. Say this
trade-off before you change it.

Stop rule. The code ends the run when it finds no goal for 10 s. For a maze where you do not
know the size, also decide a time limit, for example 10 minutes, and stop it yourself.

---

## When it breaks

These rows come from reading the code and from Lab 2. Gazebo has not confirmed them yet.
Phase 5 of the Linux prompt adds the checked rows to `RUNSHEET.md`.

| Symptom | Check | Fix |
|---|---|---|
| `Package 'r7021e_exploration' not found` | The terminal did not source the workspace | `source ~/R7021E-group6/lab3/ros2_ws/install/setup.bash` |
| `ros2 topic list` almost empty | Wrong domain in this terminal | `printenv ROS_DOMAIN_ID`, then export it |
| Launch error about `use_sim_time` | The launch arguments are not declared before the nodes | Check the order in `exploration.launch.py` |
| No map in RViz | `ros2 lifecycle get /slam_toolbox`. Is `/scan` running? | Wait for `active`. Check `use_sim_time` and the scan |
| `TF lookup map <- base_link failed`, repeated | SLAM not active, or `use_sim_time` differs between nodes, or clocks differ | `ros2 param get /slam_toolbox use_sim_time`, `view_frames`, `date` on both machines |
| Path appears, robot does not move | `ros2 topic info /cmd_vel -v`: both ends must say `TwistStamped`. Log: `no fresh scan` | Fix the type. Check `/scan` |
| Robot does not move on the real robot | Motors are off | `ros2 service call /motor_power std_srvs/srv/SetBool "{data: true}"` |
| Log: `no fresh scan, forward speed held at zero` | `/scan` is missing or old | `ros2 topic hz /scan`. On the robot, check the bringup |
| Log: `no free cell near the robot` | The robot is in unknown or wall space of the map | Move the robot to open floor |
| Many `not reachable, crossed off` | The collar closes corridors, or the map has pockets | Look at the map. Consider a smaller `inflation_radius` |
| `stalled` again and again | The robot touches a wall, or the path cuts a corner | Look at the robot. Check `waypoint_spacing` and the collar |
| `exploration finished` too early | The robot is in a closed pocket of the map | Look at RViz for frontiers. Restart from a better spot |
| Map shows doubled walls or jumps | Two `/clock` publishers, a stray `gz sim`, or loop closure | `ps aux \| grep -E "[g]z sim"`, kill. If it stays, `do_loop_closing: false` |
| Robot jumps in RViz, `TF_OLD_DATA` | A `gz sim` from a former run | Same: find and kill it |
| Two nodes with one name | The launch ran twice | `ros2 node list`, stop one launch |
| Robot keeps moving after a Ctrl-C | The last `/cmd_vel` stays active | Run the stop command |
| `ros2 param set` changes nothing | The node reads values once at start | Edit `DEFAULTS` and restart |
| Plan takes several seconds | First plan on a big map, or 15000 iterations | Read `plan_time` in the log. It is normal early in a run |
