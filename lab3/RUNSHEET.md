# Lab 3 run sheet

Robot: turtle ___   IP 192.168.50.__0   domain 3__
Mapping: turtleN -> 192.168.50.N0 -> domain 3N

Every command here ran in Gazebo on 2026-10-10, unless the line says "not tested on the robot".
When this file and `STUDY/COMMANDS.md` disagree, this file wins.

## Stop the robot

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{}"
```

Keep that pasted in a spare terminal. The follower also sends zero when you Ctrl-C the launch.
The robot has no timeout of its own: `turtlebot3_node` keeps the last command until a new one
comes. If the laptop dies, or the Wi-Fi drops, the robot keeps its last speed. Hand on the robot.

## Every terminal starts with these lines

```bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=3X
source ~/R7021E-group6/lab3/ros2_ws/install/setup.bash
```

Miss the export and the terminal sees nothing, with no error. Miss the source and you get
"Package 'r7021e_exploration' not found". Check any time with `printenv ROS_DOMAIN_ID`.

`~/.bashrc` line 125 says `ROS_DOMAIN_ID=32  # turtle4`. 32 is turtle 2. Turtle 4 is 34. Fix
the number and the comment from the robot you get, then open a new terminal.

## Build

Once per session:

```bash
cd ~/R7021E-group6/lab3/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

Pass: no error, and this lists four executables (`frontier_detector_node`, `navigation_node`,
`path_follower_node`, `task1_baseline`):

```bash
ros2 pkg executables r7021e_exploration
```

Because of `--symlink-install`, editing a `.py` file or `slam_config.yaml` needs no rebuild.
Restart the launch. Changing `setup.py` or `package.xml` does need one.

---

# Simulation

## Gazebo (terminal 1)

```bash
ros2 launch ~/R7021E-group6/lab3/sim/maze_world.launch.py world:=lab3_maze_small
```

Worlds: `lab3_maze_small` (4 m), `lab3_maze` (7.2 m), `lab3_maze_narrow` (doors 0.30, 0.40 and
0.50 m wide), `lab3_maze_blind_a`, `lab3_maze_blind_b`. Add `gui:=false` for no Gazebo window.

Checks, with what they printed here:

```bash
ros2 topic list
```

`/clock /cmd_vel /imu /joint_states /odom /scan /tf` and the two ROS ones.

```bash
ros2 topic echo /clock --once
```

A time that grows between two calls.

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

A translation near `[0, 0, 0.01]`.

```bash
ros2 topic info /cmd_vel -v
```

`Topic type: geometry_msgs/msg/TwistStamped`, subscriber `ros_gz_bridge`. The follower sends
TwistStamped too. If one end says `Twist`, the robot does not move and nothing errors.

`ros2 topic hz /scan` printed nothing on this laptop. Count scans in 10 s instead. It printed
45 here (4.5 per second of wall time, the sim ran a little slower than real time):

```bash
timeout 10 ros2 topic echo /scan | grep -c "^header:"
```

## Stop Gazebo

Ctrl-C does not stop all of it. Here two `gz sim` processes were still running every time.

```bash
ps aux | grep -E "[g]z sim"
```

```bash
pkill -f "gz sim"
```

Do this before every new launch. Two simulators on two clocks make the robot jump in RViz.

## Task 1 (terminal 2)

The spawn cell in `lab3_maze_small` has a wall 0.35 m ahead. Task 1 drives 0.5 m, so spawn one
cell back:

```bash
ros2 launch ~/R7021E-group6/lab3/sim/maze_world.launch.py world:=lab3_maze_small x_pose:=-0.8
```

```bash
ros2 launch r7021e_exploration exploration.launch.py use_sim_time:=true task1:=true
```

Pass: the log prints `leg 1`, `leg 2`, ..., about 6.5 s per leg. These three print `True`:

```bash
ros2 param get /path_follower_node use_sim_time
```

```bash
ros2 param get /slam_toolbox use_sim_time
```

```bash
ros2 param get /task1_baseline use_sim_time
```

`ros2 node list` has no `navigation_node`. `ros2 lifecycle get /slam_toolbox` says `active`.

## Full exploration

```bash
ros2 launch r7021e_exploration exploration.launch.py use_sim_time:=true 2>&1 | tee ~/R7021E-group6/lab3/results/<run>/launch.log
```

What it printed here: see `results/*/summary.md`. In the small maze a full run is about 4 to 5
minutes and ends with `exploration finished`. In the big maze it is about 20 minutes.

The planner prints one line for each plan:

```
t=82.0 goal=(0.89, 1.06) H=-4.00 L=0.57 I=46 turn=0.19 candidates=5 known=11.2m2 plan_time=0.15s
```

Other lines: `arrived at`, `stalled, crossing off the goal`, `path is old, planning again`,
`trying N stalled goals again`, `not reachable, crossed off`, `exploration finished`. The
follower prints `blocked ahead at 0.17 m, backing off` when it reverses 0.10 m.

## Recording

Make a folder per run, start the recorder first, then the launch:

```bash
mkdir -p ~/bags ~/R7021E-group6/lab3/results/<run>
```

```bash
ros2 bag record -o ~/bags/<run> /map /frontiers /path /cmd_vel /odom /tf /tf_static /scan /clock /rrt_tree /goal_marker /inflated_map
```

Stop the launch first. Then Ctrl-C the recorder **once** and wait until it prints
`Recording stopped`. Then check duration and message counts:

```bash
ros2 bag info ~/bags/<run>
```

The output folder must not exist yet, or the recorder refuses to start. Use a new name.

## Demo video from a bag

No ROS graph needed, no RViz. Works with the network off.

```bash
source /opt/ros/jazzy/setup.bash
```

```bash
python3 ~/R7021E-group6/lab3/video/bag_to_video.py ~/bags/<run> --speed 10 --out ~/<run>.mp4
```

It needs `/map /tf /path /rrt_tree /goal_marker /inflated_map /frontiers` in the bag. A 4 minute
run takes about 15 s to render. `--out x.gif` makes a GIF if ffmpeg is missing.

---

# Real robot (not tested on the robot)

Steps from `LAB2_RUNSHEET.md` and Lab 1. The lab 3 stack itself has only run in Gazebo and on a
replayed Lab 1 robot bag.

## Before you start

1. Battery charged. Laptop charged. Laptop suspend off.
2. Look at the maze walls. The laser is about 0.17 m above the floor. A wall lower than that
   is invisible to SLAM and to the follower.
3. Measure the narrowest passage the robot must drive. The planner needs about 0.40 m between
   wall faces with the normal `inflation_radius` 0.105 m. See "Changing a value".
4. Start spot: nearest wall more than 0.3 m away, 0.5 m free ahead and behind. The map starts
   here.

## SSH terminal, on the robot, no export

```bash
ssh turtle@192.168.50.X0
```

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

## Set the robot clock (laptop terminal)

The Lab 1 robot clock was 325 days behind the laptop (from the Lab 1 bags). The stack still
runs, a replay of a Lab 1 bag proved it, but a bag played back into RViz can drop data. Set
it before you record. You type the robot's sudo password:

```bash
ssh -t turtle@192.168.50.X0 "sudo date -u -s '$(date -u +%Y-%m-%dT%H:%M:%S)'"
```

Check it: `date +%T` on the laptop and `ssh turtle@192.168.50.X0 date +%T` differ by under 1 s.

## Laptop checks

```bash
ros2 topic list
```

```bash
ros2 topic info /cmd_vel -v
```

TwistStamped and one subscriber, `turtlebot3_node`.

Ten seconds that separate a robot problem from a code problem:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.05}}}"
```

The robot creeps forward. Stop it with the stop command.

## Task 1 and the full run

```bash
ros2 launch r7021e_exploration exploration.launch.py task1:=true
```

```bash
ros2 launch r7021e_exploration exploration.launch.py 2>&1 | tee ~/R7021E-group6/lab3/results/<run>/launch.log
```

No `use_sim_time`. Recorder first, same line as above. A phone video of the robot as well.

---

# When it breaks

| symptom | command that shows it | fix |
|---|---|---|
| `Package 'r7021e_exploration' not found` | `printenv AMENT_PREFIX_PATH` has no `lab3/ros2_ws` | `source ~/R7021E-group6/lab3/ros2_ws/install/setup.bash` |
| `ros2 topic list` nearly empty | `printenv ROS_DOMAIN_ID` | export the same domain in every terminal |
| Robot never moves, topics look fine | `ros2 topic info /cmd_vel -v` | both ends must say TwistStamped |
| Robot jumps in RViz, `TF_OLD_DATA` | `ps aux \| grep -E "[g]z sim"` | `pkill -f "gz sim"`, start again |
| `TF lookup map <- base_link failed`, repeated | `ros2 lifecycle get /slam_toolbox`, `ros2 param get /slam_toolbox use_sim_time` | wait for `active`. Gazebo needs `use_sim_time:=true`, the robot needs it left out |
| Follower prints `no map -> base_link transform, holding still` | `ros2 run tf2_tools view_frames` | as above. The robot is held at zero meanwhile |
| Map smears, walls doubled, robot pose jumps by a metre | the map in RViz | loop closure is already off. If it still happens, drive slower (`max_v` 0.10) |
| `blocked ahead ..., backing off` many times in one place | the log | normal at tight corners. It moves 0.10 m back and tries again |
| `stalled, crossing off the goal`, then `exploration finished` early | `/frontiers` in RViz: green cells left? | the goal comes back up to 2 times by itself. If the corridor is narrow, see "Changing a value" |
| `not reachable, crossed off` for a corridor you can see | measure it | narrower than about 0.40 m. Lower `inflation_radius` |
| Bag folder has no `metadata.yaml` | `ls ~/bags/<run>` | the recorder was killed, not stopped. Ctrl-C once and wait |
| Robot clips corners on the real robot | the bag: Wi-Fi gaps | the laptop gets data up to 3.6 s late on bad Wi-Fi (Lab 1 bags). Drive slower: `max_v` 0.10 |

---

# Changing a value

Every value is a default at the top of its node. Edit, save, restart the launch. No rebuild.

| What | Where | Normal | Why you would change it |
|---|---|---|---|
| `inflation_radius` | `DEFAULTS` in `navigation_node.py` | 0.105 m (3 cells, 0.15 m collar) | 0.09 m gives 2 cells and opens corridors from about 0.30 m |
| `info_weight` | `DEFAULTS` in `navigation_node.py` | 0.10 m per frontier cell | Task 4 |
| `max_v` | `declare_parameter` in `path_follower_node.py` | 0.15 m/s | 0.10 on bad Wi-Fi |
| `max_w`, `kp_yaw` | same file | 0.6 rad/s, 1.0 | faster turns slip, and the SLAM pose drifts |
| `look_ahead` | same file | 0.12 m | 0.20 cut corners into walls |
| `do_loop_closing` | `config/slam_config.yaml` | false | true gave false closures in the Gazebo maze |

Read a value back from a running node:

```bash
ros2 param get /navigation_node inflation_radius
```

`ros2 param set` changes nothing. The nodes read their values once at start.

`git diff` must be empty after an experiment.

---

# Read things with no internet

```bash
ros2 interface show geometry_msgs/msg/TwistStamped
```

```bash
ros2 interface show nav_msgs/msg/OccupancyGrid
```

```bash
ros2 topic info /cmd_vel -v
```

```bash
ros2 param list /navigation_node
```

```bash
ros2 run tf2_tools view_frames
```

That writes `frames.pdf` in the current folder.

```bash
ros2 topic pub --help
```
