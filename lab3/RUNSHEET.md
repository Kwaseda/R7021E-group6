# Lab 3 run sheet

Robot: turtle ___   IP 192.168.50.__0   domain 3__
Mapping used by this sheet: turtleN -> 192.168.50.N0 -> domain 3N.

**Before the demo, confirm the number printed on the actual robot.** Do not copy the
Turtlebot 4 example below blindly: this laptop's `~/.bashrc` has previously set domain 32
while labeling it turtle4. For a confirmed turtle4, the mapping above gives IP
`192.168.50.40` and domain `34`. Set the correct domain manually in every laptop terminal
and again in the robot's SSH terminal; do not trust a pre-existing shell setting.

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

## Offline build (once after checkout, or after package/code changes)

The laptop must already have ROS Jazzy, colcon, the package dependencies, and the TurtleBot3
and Gazebo packages installed. This build does not fetch dependencies from the internet.

```bash
source /opt/ros/jazzy/setup.bash
```

```bash
cd ~/R7021E-group6/lab3/ros2_ws
```

```bash
colcon build --symlink-install
```

You should see `Summary: 1 package finished`. After a successful build, source the workspace
in every laptop terminal using the start lines above. With `--symlink-install`, editing a
package `.py` file needs a node restart but not another build.

## Laptop terminal setup

Every laptop terminal gets these lines first. If building for the first time, complete the
Offline build section above before sourcing the workspace. Replace `34` with 3 times the
confirmed robot number.

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
source ~/R7021E-group6/lab3/ros2_ws/install/setup.bash
```

```bash
printenv ROS_DOMAIN_ID
```

It must print the same confirmed number in every laptop terminal. A different domain looks
like an empty ROS graph and gives no error. Re-run the setup steps in every new terminal.

---

# In the hall: the real robot

Open five terminals. Give each one a name in its tab (right click on the tab, or just remember
the order). Do them in this order.

### Terminal 1: robot bringup (SSH, on the robot)

For confirmed turtle4 only (replace the IP for any other robot):

```bash
ssh turtle@192.168.50.40
```

Password `turtle`. In the robot shell, set up ROS and the same domain as the laptop.
Replace `34` if the confirmed robot is not turtle4:

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
printenv ROS_DOMAIN_ID
```

Only after that check prints the same domain as the laptop, start bringup:

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

You should see it print lines about the motors and the lidar and then keep running. Leave it.
It runs until you Ctrl-C it or the robot turns off.

### Terminal 2: checks (laptop)

Run the laptop terminal setup above first. For confirmed turtle4 only, set the robot clock
over the local lab Wi-Fi; this does not require internet access. Replace the IP if needed.

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

Run the laptop terminal setup above, then paste the stop command (top of this file). Do not
press Enter yet. If anything unexpected moves, press Enter once to send zero velocity.

### Terminal 4: the bag recorder (laptop)

Run the laptop terminal setup above. Start a new recorder for each task/run and use a new
output name each time. For example, Task 1:

```bash
mkdir -p ~/bags && ros2 bag record -o ~/bags/hall-task1 /map /frontiers /path /cmd_vel /odom /tf /tf_static /scan /rrt_tree /goal_marker /inflated_map
```

Start this before Terminal 5. It prints `Subscribed to topic` lines and waits.

### Terminal 5: the stack (laptop)

Run the laptop terminal setup above. Task 1 first (needs 0.5 m free floor ahead of the
robot):

```bash
ros2 launch r7021e_exploration exploration.launch.py task1:=true
```

You should see `leg 1`, `leg 2`, and the robot drives 0.5 m forward and back. Ctrl-C once.

Stop the Task 1 bag recorder with Ctrl-C once, wait for `Recording stopped`, and check it
before starting Task 2:

```bash
ros2 bag info ~/bags/hall-task1
```

### Task 2: plan to a known goal using the RViz tool

Start a separate bag before launching the planner:

```bash
ros2 bag record -o ~/bags/hall-task2 /map /frontiers /path /cmd_vel /odom /tf /tf_static /scan /rrt_tree /goal_marker /inflated_map
```

In Terminal 5, launch the normal stack. Do not add `use_sim_time` on the real robot:

```bash
ros2 launch r7021e_exploration exploration.launch.py
```

Wait for RViz to show the map. Its Fixed Frame is already configured as `map`; if RViz has
a red Fixed Frame error, select `map` in **Global Options → Fixed Frame**. In the RViz top
toolbar, click **2D Goal Pose** (the goal-arrow tool, not **Set Initial Pose** or **Publish
Point**). On the visible map, left-click a known-free location down a corridor, drag a short
distance to set the arrow, then release. The arrow direction is ignored by this planner.

The click publishes `/goal_pose`. Confirm Terminal 5 prints `goal from RViz: (x, y)`, then
look for the green `/path`, teal RRT* tree, red goal marker, and the robot driving toward
the selected point. Choose a known-free point away from walls; a click in a wall or outside
known free space can produce `not reachable` instead. When the `arrived at` line for that
goal appears, stop Terminal 5 with Ctrl-C once before the planner resumes its automatic
exploration. Stop the recorder with Ctrl-C once, wait for `Recording stopped`, then check:

```bash
ros2 bag info ~/bags/hall-task2
```

No internet connection is needed for this RViz tool; the map, planner and `/goal_pose` topic
are all local ROS traffic on the lab network.

### Task 5: full exploration

Start a fresh recorder with a unique bag name:

```bash
ros2 bag record -o ~/bags/hall-task5 /map /frontiers /path /cmd_vel /odom /tf /tf_static /scan /rrt_tree /goal_marker /inflated_map
```

Then launch the same normal stack as above and let it explore. Do not click a goal for this
run. Stop the launch and recorder as described below, then check:

```bash
ros2 bag info ~/bags/hall-task5
```

### Ending a run

1. Wait for `exploration finished` in Terminal 5, or stop when you decide.
2. Terminal 5: Ctrl-C once. The robot stops.
3. Terminal 4: Ctrl-C once, wait for `Recording stopped`.
4. Check the bag:

```bash
ros2 bag info ~/bags/hall-task2
```

Duration and message counts must not be zero.

---

# At home: Gazebo

Three terminals. Start lines in each.

### Terminal 1: Gazebo

In this terminal too, source ROS and set the same domain used by the other laptop terminals:

```bash
source /opt/ros/jazzy/setup.bash
```

```bash
export TURTLEBOT3_MODEL=burger
```

```bash
export ROS_DOMAIN_ID=34
```

Replace `34` with the confirmed robot domain. Do not use a different domain for Gazebo.

Check for leftover Gazebo processes before every launch. Ctrl-C does not always stop them:

```bash
ps aux | grep -E "[g]z sim"
```

If that lists stale Gazebo server or GUI processes, note their PID(s) and stop only those
specific processes. For example, replace `12345` with one PID shown above:

```bash
kill 12345
```

Check again with `ps aux | grep -E "[g]z sim"` before launching. Do not kill a PID unless
you have confirmed it is a leftover `gz sim` process.

```bash
ros2 launch ~/R7021E-group6/lab3/sim/maze_world.launch.py world:=lab3_maze_small
```

Gazebo alone does nothing. The robot stands still until Terminal 3 runs. That is correct.
For Task 1, use the small world with `x_pose:=-0.8` (a wall is 0.35 m ahead of the normal
start):

```bash
ros2 launch ~/R7021E-group6/lab3/sim/maze_world.launch.py world:=lab3_maze_small x_pose:=-0.8
```

Worlds: `lab3_maze_small` (4 m), `lab3_maze` (7.2 m), `lab3_maze_narrow`, `lab3_maze_blind_a`
(5.6 m), `lab3_maze_blind_b` (7.2 m). Add `gui:=false` to hide the Gazebo window.

### Terminal 2: checks, then the bag

Run the laptop terminal setup above first. Use a new bag name for each task/run.

```bash
ros2 topic info /cmd_vel -v
```

`Publisher count: 0` is normal before Terminal 3. After Terminal 3 starts it is 1.

```bash
mkdir -p ~/bags && ros2 bag record -o ~/bags/sim-1 /map /frontiers /path /cmd_vel /odom /tf /tf_static /scan /clock /rrt_tree /goal_marker /inflated_map
```

### Terminal 3: the stack

Run the laptop terminal setup above first.

```bash
ros2 launch r7021e_exploration exploration.launch.py use_sim_time:=true
```

`use_sim_time:=true` only in Gazebo. For Task 1, add `task1:=true`. For Task 2, start the
normal stack and use the **2D Goal Pose** tool in RViz exactly as described above for the
hall: click the toolbar button, click a known-free map location, drag a short arrow and
release. Confirm `goal from RViz` in the navigation log. If the goal is not reachable, pick
a different visible free location away from walls. For Task 3, compare inflation settings
only in Gazebo: setting `inflation_radius` to `0.0` removes the planner's wall clearance and
is not a safe hardware test. This launch file does not expose node parameters as
`inflation_radius:=...` launch arguments; edit the node default in `navigation_node.py`,
restart the stack, and restore it afterwards. For Task 4, compare `info_weight` values in
Gazebo and record each run separately; change the node default in that same file between
runs. Task 5 is the unclicked full exploration run.

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
offline. Replace `hall-task5` below with the bag name for the run you want to document.
These scripts read the bag directly; they do not start ROS nodes or replay it.

```bash
source /opt/ros/jazzy/setup.bash
```

```bash
source ~/R7021E-group6/lab3/ros2_ws/install/setup.bash
```

```bash
python3 ~/R7021E-group6/lab3/video/bag_to_report.py ~/bags/hall-task5 --out-dir ~/bags/hall-task5-report
```

This writes two PNGs:

- `hall-task5_map.png`: final SLAM occupancy map, planner-blocked/inflated cells, frontiers, the last
  non-trivial RRT* tree/path/goal, and the robot's recorded map-frame trail with start/end.
  Its legend names each layer. In `/inflated_map`, 100 means a known cell is not traversable
  by the planner (occupied or inside the wall-clearance inflation); 0 means otherwise.
  It is not itself a safe-cell mask.
- `hall-task5_metrics.png`: known mapped area and `/cmd_vel` linear/angular commands against
  elapsed bag receive time. Known area is not a completion percentage: the real maze's
  ground-truth area is not recorded in the bag.

Make a compact animated GIF of the changing map and planner state:

```bash
python3 ~/R7021E-group6/lab3/video/bag_to_video.py ~/bags/hall-task5 --speed 30 --fps 5 --out ~/bags/hall-task5-report/hall-task5.gif
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
| Robot invisible but laptop domain looks right | `printenv ROS_DOMAIN_ID` in the robot SSH shell too | set the same verified domain in both laptop and robot shells |
| Robot does not move, Gazebo open | `ros2 topic info /cmd_vel -v`: publisher 0 | start Terminal 3 (the stack) |
| Robot does not move, stack running | `ros2 topic info /cmd_vel -v` | both ends must say TwistStamped |
| Robot does not move on hardware | Terminal 1 still running? | restart bringup. If still nothing: `ros2 service call /motor_power std_srvs/srv/SetBool "{data: true}"` |
| Robot jumps in RViz, `TF_OLD_DATA` | `ps aux \| grep -E "[g]z sim"` | replace `12345` with a stale Gazebo PID and run `kill 12345`, then verify the list is clear |
| `TF lookup` warnings repeat | `ros2 lifecycle get /slam_toolbox` | wait for `active`. Gazebo needs `use_sim_time:=true`, the robot needs it left out |
| Robot circles one spot, `I` stays 1 to 6 | the log | current `min_gain` is 7; the threshold was raised after low-gain slivers kept being selected |
| Finished with green frontier left in RViz | look where it is | behind a wall or in a passage under 0.40 m: normal. Otherwise restart the launch |
| Many `backing off` lines in one place | the log | normal at tight corners |
| Bag folder has no `metadata.yaml` | `ls ~/bags/hall-task5` | the recorder was killed. Ctrl-C once and wait next time |
| Robot clips corners on bad Wi-Fi | how laggy RViz is | drive slower: `max_v` 0.10 (see below) |

# Changing a value

1. Ctrl-C the launch.
2. Edit the number in the file below and save.
3. Start the launch again. No rebuild.
4. Check it: `ros2 param get /navigation_node inflation_radius`.
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
