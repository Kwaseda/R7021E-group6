# Lab 3 run sheet

Robot: turtle ___   IP 192.168.50.__0   domain 3__
Mapping: turtleN -> 192.168.50.N0 -> domain 3N. Turtle 4 is 192.168.50.40, domain 34.

Nothing here needs the internet. The router in the hall is a local network. If a step fails,
the "When it breaks" table at the bottom has the command that finds the cause.

---

## The stop command

Paste this in its own terminal before anything moves, and leave it there:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{}"
```

The robot keeps its last command until it gets a new one. If anything goes wrong, press Enter
on this terminal (after the up arrow). Then Ctrl-C the launch.

## The three start lines

Every laptop terminal gets these three lines first. Every time. Change 34 to your robot.

```bash
export TURTLEBOT3_MODEL=burger
```

```bash
export ROS_DOMAIN_ID=34
```

```bash
source ~/R7021E-group6/lab3/ros2_ws/install/setup.bash
```

Check: `printenv ROS_DOMAIN_ID` prints your number. If one terminal has a different number, it
sees nothing and gives no error.

## Build (once per session)

```bash
cd ~/R7021E-group6/lab3/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

You should see `Summary: 1 package finished`. After this, editing a `.py` file needs no rebuild,
only a restart of the launch.

---

# In the hall: the real robot

Open five terminals. Give each one a name in its tab (right click on the tab, or just remember
the order). Do them in this order.

### Terminal 1: robot bringup (SSH, on the robot)

```bash
ssh turtle@192.168.50.40
```

Password `turtle`. No start lines here, this terminal is the robot itself.

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

You should see it print lines about the motors and the lidar and then keep running. Leave it.
It runs until you Ctrl-C it or the robot turns off.

### Terminal 2: checks (laptop)

Three start lines first. Then set the robot clock. You type the robot's sudo password:

```bash
ssh -t turtle@192.168.50.40 "sudo date -u -s '$(date -u +%Y-%m-%dT%H:%M:%S)'"
```

Then check the robot is visible:

```bash
ros2 topic list
```

You should see `/scan`, `/odom`, `/tf`, `/cmd_vel`. If the list is short, the domain is wrong.

```bash
ros2 topic info /cmd_vel -v
```

You should see `TwistStamped` and `Subscription count: 1` (that is `turtlebot3_node`).

Prove the robot moves without our code. It creeps forward:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.05}}}"
```

Stop it:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{}"
```

### Terminal 3: the stop command (laptop)

Three start lines, then paste the stop command (top of this file). Do not press Enter yet.

### Terminal 4: the bag recorder (laptop)

Three start lines. Then, with a new name for each run:

```bash
mkdir -p ~/bags && ros2 bag record -o ~/bags/hall-1 /map /frontiers /path /cmd_vel /odom /tf /tf_static /scan /rrt_tree /goal_marker /inflated_map
```

Start this before Terminal 5. It prints `Subscribed to topic` lines and waits.

### Terminal 5: the stack (laptop)

Three start lines. Task 1 first (needs 0.5 m free floor ahead of the robot):

```bash
ros2 launch r7021e_exploration exploration.launch.py task1:=true
```

You should see `leg 1`, `leg 2`, and the robot drives 0.5 m forward and back. Ctrl-C once.

Then the full run:

```bash
ros2 launch r7021e_exploration exploration.launch.py
```

No `use_sim_time` on the robot. RViz opens. Within about 20 s: the map, the first plan line,
and the robot moves. Start the phone video and the screen recording now.

### Ending a run

1. Wait for `exploration finished` in Terminal 5, or stop when you decide.
2. Terminal 5: Ctrl-C once. The robot stops.
3. Terminal 4: Ctrl-C once, wait for `Recording stopped`.
4. Check the bag:

```bash
ros2 bag info ~/bags/hall-1
```

Duration and message counts must not be zero.

---

# At home: Gazebo

Three terminals. Start lines in each.

### Terminal 1: Gazebo

Clean up the last run first. Ctrl-C does not stop all of Gazebo:

```bash
pkill -f "gz sim"
```

```bash
ros2 launch ~/R7021E-group6/lab3/sim/maze_world.launch.py world:=lab3_maze_small
```

Gazebo alone does nothing. The robot stands still until Terminal 3 runs. That is correct.
For Task 1 add `x_pose:=-0.8` (a wall is 0.35 m ahead of the normal start).
Worlds: `lab3_maze_small` (4 m), `lab3_maze` (7.2 m), `lab3_maze_narrow`, `lab3_maze_blind_a`
(5.6 m), `lab3_maze_blind_b` (7.2 m). Add `gui:=false` to hide the Gazebo window.

### Terminal 2: checks, then the bag

```bash
ros2 topic info /cmd_vel -v
```

`Publisher count: 0` is normal before Terminal 3. After Terminal 3 starts it is 1.

```bash
mkdir -p ~/bags && ros2 bag record -o ~/bags/sim-1 /map /frontiers /path /cmd_vel /odom /tf /tf_static /scan /clock /rrt_tree /goal_marker /inflated_map
```

### Terminal 3: the stack

```bash
ros2 launch r7021e_exploration exploration.launch.py use_sim_time:=true
```

`use_sim_time:=true` only in Gazebo. Task 1: add `task1:=true`.

### Reading the log

One line per plan:

```
current_time=82.0 goal=(0.89, 1.06) H=-4.00 L=0.57 I=46 turn=0.19 nodes=1500 mapped_area=11.2m² plan_time=0.25s
```

`I` is how many frontier cells the goal sees. Real goals have I of 10 or more. A goal needs at
least 7 (`min_gain`, current source default). When no leaf sees 7 cells for 10 plans in a row
after at least one successful plan, it prints `exploration finished` and holds the robot still.
Other lines: `arrived at`,
`stalled, planning again`, `path is old, planning again`, `goal from RViz`, `not reachable`.
After Ctrl-C the frontier detector may print a traceback. That is a shutdown race and harmless.

## Demo video from a bag

```bash
python3 ~/R7021E-group6/lab3/video/bag_to_video.py ~/bags/sim-1 --speed 10 --out ~/sim-1.mp4
```

## Report figures and GIF from a hall bag

After the bag has stopped cleanly and `ros2 bag info` shows messages, make the figures
offline. These scripts read the bag directly; they do not start ROS nodes or replay it.

```bash
source /opt/ros/jazzy/setup.bash && source ~/R7021E-group6/lab3/ros2_ws/install/setup.bash
```

```bash
python3 ~/R7021E-group6/lab3/video/bag_to_report.py ~/bags/hall-1 --out-dir ~/bags/hall-1-report
```

This writes two PNGs:

- `hall-1_map.png`: final SLAM occupancy map, planner-safe cells, frontiers, the last
  non-trivial RRT* tree/path/goal, and the robot's recorded map-frame trail with start/end.
  Its legend names each layer. `/inflated_map` is the planner's safe-cell mask (100 means
  safe), not a picture of the wall padding itself.
- `hall-1_metrics.png`: known mapped area and `/cmd_vel` linear/angular commands against
  elapsed bag receive time. Known area is not a completion percentage: the real maze's
  ground-truth area is not recorded in the bag.

Make a compact animated GIF of the changing map and planner state:

```bash
python3 ~/R7021E-group6/lab3/video/bag_to_video.py ~/bags/hall-1 --speed 30 --fps 5 --out ~/bags/hall-1-report/hall-1.gif
```

The GIF has the same layer legend on every frame. Higher `--speed` or lower `--fps`
reduces the number of frames and file size. Copy the PNGs and GIF into the report folder
after checking them; keep the original bag unchanged.

---

# When it breaks

| Symptom | Command that shows it | Fix |
|---|---|---|
| `Package 'r7021e_exploration' not found` | none needed | run the `source` start line in this terminal |
| `ros2 topic list` nearly empty | `printenv ROS_DOMAIN_ID` | export the same domain in every terminal |
| Robot does not move, Gazebo open | `ros2 topic info /cmd_vel -v`: publisher 0 | start Terminal 3 (the stack) |
| Robot does not move, stack running | `ros2 topic info /cmd_vel -v` | both ends must say TwistStamped |
| Robot does not move on hardware | Terminal 1 still running? | restart bringup. If still nothing: `ros2 service call /motor_power std_srvs/srv/SetBool "{data: true}"` |
| Robot jumps in RViz, `TF_OLD_DATA` | `ps aux \| grep -E "[g]z sim"` | `pkill -f "gz sim"`, start again |
| `TF lookup` warnings repeat | `ros2 lifecycle get /slam_toolbox` | wait for `active`. Gazebo needs `use_sim_time:=true`, the robot needs it left out |
| Robot circles one spot, `I` stays 1 to 6 | the log | current `min_gain` is 7; the threshold was raised after low-gain slivers kept being selected |
| Finished with green frontier left in RViz | look where it is | behind a wall or in a passage under 0.40 m: normal. Otherwise restart the launch |
| Many `backing off` lines in one place | the log | normal at tight corners |
| Bag folder has no `metadata.yaml` | `ls ~/bags/<run>` | the recorder was killed. Ctrl-C once and wait next time |
| Robot clips corners on bad Wi-Fi | how laggy RViz is | drive slower: `max_v` 0.10 (see below) |

# Changing a value

1. Ctrl-C the launch.
2. Edit the number in the file below and save.
3. Start the launch again. No rebuild.
4. Check it: `ros2 param get /navigation_node <name>`.
5. Afterwards put it back. `git diff` must be empty.

| What | File, place | Normal | When to change |
|---|---|---|---|
| `inflation_radius` | `navigation_node.py`, `DEFAULTS` | 0.105 | 0.09 if a must-pass gap is 0.30 to 0.40 m |
| `info_weight` | same | 0.10 | Task 4 test: 0.0 and 0.30 |
| `min_gain` | same | 7 | Raising it further rejects more small frontiers; compare coverage before changing |
| `max_v` | `path_follower_node.py`, `declare_parameter` | 0.15 | 0.10 on bad Wi-Fi |

`ros2 param set` does not change anything. The nodes read their values once at start.

# Read things with no internet

```bash
ros2 interface show geometry_msgs/msg/TwistStamped
```

```bash
ros2 param list /navigation_node
```

```bash
ros2 run tf2_tools view_frames
```

That writes `frames.pdf` in the current folder.
