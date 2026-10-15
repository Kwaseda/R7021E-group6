# Project run sheet

Commands that are known to work, in order. Dominic types them. Every new terminal needs ROS
sourced and the same `ROS_DOMAIN_ID`.

---

## Gate 1: the copied node reproduces Lab 2 in Gazebo

Plain Burger, empty world, no plank yet. Proves the copy builds, registers and drives before
anything new is added.

### 0. Before every launch (trap 3: stray Gazebo)

```bash
pgrep -af "gz sim"
```

If anything is listed:

```bash
pkill -f "gz sim"
```

### 1. Build once (T1)

`--symlink-install` means editing `mpc_node.py` later needs no rebuild, only a restart of the node.

```bash
cd ~/R7021E-group6/project/ros2_ws && colcon build --symlink-install
```

### 2. Every terminal (T1-T4)

```bash
source ~/R7021E-group6/project/ros2_ws/install/setup.bash
```

```bash
printenv ROS_DOMAIN_ID
```

All four must print the same number.

### 3. T1: Gazebo, the sim's "bringup"

```bash
export TURTLEBOT3_MODEL=burger
```

```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### 4. T2: the MPC node, task 4 (circle)

`n_horizon:=20` matches what the robot ran in Lab 2 (the node default is still 100 and gets
fixed after the horizon sweep). `use_sim_time:=true` because `/odom` is stamped with Gazebo's clock.

```bash
ros2 run r7021e_project mpc_node --ros-args -p task:=4 -p n_horizon:=20 -p use_sim_time:=true
```

### 5. T3: RViz

```bash
rviz2 -d ~/R7021E-group6/project/ros2_ws/install/r7021e_project/share/r7021e_project/rviz/project.rviz
```

Fixed Frame `odom`. You should see the orange obstacle and its keep-out ring, and the predicted
path running ahead of the robot.

### 6. T4: record (start before T2 if you want the drive-on)

```bash
ros2 bag record -o ~/R7021E-group6/project/bags/gate1_t4 /odom /cmd_vel /mpc_prediction /tf /tf_static
```

Ctrl-C **once** in T4 after two laps (about 130 s), and wait for it to finish. Then:

```bash
ros2 bag info ~/R7021E-group6/project/bags/gate1_t4
```

### 7. Plot and compare

```bash
cd ~/R7021E-group6/project/analysis && python3 plot_run.py ../bags/gate1_t4 --task 4
```

**Pass:** no `solver failed` lines in T2, obstacle closest approach about 0.225 m, mean
tracking error after drive-on a few mm. The desk run gave 0.2249 m and 0.0026 m.

### 8. Task 3 (slalom), same terminals

Stop T2 (Ctrl-C), reset the robot by restarting T1 (after the `pgrep` check), then:

```bash
ros2 run r7021e_project mpc_node --ros-args -p task:=3 -p n_horizon:=20 -p use_sim_time:=true
```

```bash
ros2 bag record -o ~/R7021E-group6/project/bags/gate1_t3 /odom /cmd_vel /new_position /mpc_prediction /tf /tf_static
```

Goal on the axis between the two obstacles, which Lab 2 got wrong on the floor:

```bash
ros2 topic pub /new_position geometry_msgs/msg/Pose "{position: {x: 1.8, y: 0.0}}" --times 6 --rate 2
```

```bash
cd ~/R7021E-group6/project/analysis && python3 plot_run.py ../bags/gate1_t3 --task 3
```

**Pass:** the robot weaves between both obstacles and stays within about 0.12 m of the x-axis.
Both obstacles are passed at close to their keep-out radius (0.255 m). The desk run gave 0.2541
and 0.2549 m.
