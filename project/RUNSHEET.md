# Project run sheet

Commands that are known to work, in order. Dominic types them. Every new terminal needs ROS
sourced and the same `ROS_DOMAIN_ID`.

---

## Gate 1: the copied node reproduces Lab 2 in Gazebo

Plain Burger, empty world, no plank yet. Proves the copy builds, registers and drives before
anything new is added.

> **Historical.** These commands are for the verbatim Lab 2 copy at commit `a610d80`. The
> project node no longer has `task` or `plot_run.py --task`. To rerun a Lab 2 task on today's
> node: `project_sim.launch.py scenario:=lab2_t4 plank:=false`, then
> `mpc_node --ros-args -p scenario:=lab2_t4 -p footprint:=burger -p use_sim_time:=true`.

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

### What went wrong in Dominic's gate 1 t3, and the fix

The goal never reached the node, so every command was zero. Publish the goal only **after**
T2 prints its `scenario ... | footprint ...` line, and from a terminal whose
`printenv ROS_DOMAIN_ID` matches. T2 must answer with `NEW GOAL x=1.800 y=0.000`. If it
doesn't, the robot will sit still, and that's why.

---

## Project runs: plank Burger, scenarios 1 and 2

Everything below uses the project's own launch file, which is the sim's equivalent of
`turtlebot3_bringup`. One terminal per program, same as on the robot.

### 0. Before every launch

```bash
pgrep -af "gz sim"
```

```bash
pkill -f "gz sim"
```

### 1. Build (after any change to launch/, worlds/, rviz/ or setup.py; node edits don't need it)

```bash
cd ~/R7021E-group6/project/ros2_ws && colcon build --symlink-install
```

### 2. Every terminal

```bash
source ~/R7021E-group6/project/ros2_ws/install/setup.bash
```

```bash
printenv ROS_DOMAIN_ID
```

### 3. T1: Gazebo with the scenario's world (the "bringup")

`gui:=false` for any run whose solve time you will report: the Gazebo window triples it.

```bash
ros2 launch r7021e_project project_sim.launch.py scenario:=s2_030
```

### 4. T2: the MPC node, **same scenario name** (Q13)

```bash
ros2 run r7021e_project mpc_node --ros-args -p scenario:=s2_030 -p footprint:=three -p use_sim_time:=true
```

For the pair, Ctrl-C T2 and T1, run the `pgrep` check, start T1 again, and run T2 with
`footprint:=one`.

### 5. T3: RViz, on sim time

```bash
rviz2 -d ~/R7021E-group6/project/ros2_ws/install/r7021e_project/share/r7021e_project/rviz/project.rviz --ros-args -p use_sim_time:=true
```

You should see the Burger model, the orange plank with the blue discs the controller believes in,
the obstacles with their keep-out rings, the red box and the green predicted path.

### 6. T4: record, before starting T2

```bash
ros2 bag record -o ~/R7021E-group6/project/bags/my_s2_030_three /odom /ground_truth /cmd_vel /mpc_prediction /mpc_solve_time /tf /tf_static
```

### 7. Grade it

```bash
cd ~/R7021E-group6/project/analysis && python3 plot_run.py ../bags/my_s2_030_three --scenario s2_030 --footprint three
```

Pair plot, two bags on the same axes:

```bash
python3 plot_run.py ../bags/s2_050_one ../bags/s2_050_three --scenario s2_050 -o ../plots/s2_050_pair.png
```

### Scenario names

| name | what | goal |
|---|---|---|
| `s1a` | circle r 0.8, obstacle (0, 0.62) r 0.12, one lap after 8 s drive-on | path |
| `s1b` | figure-8 size 0.8, obstacle (0, 0.36) r 0.08, one lap after 2 s | path |
| `s2_060` `s2_050` `s2_040` `s2_034` `s2_030` `s2_028` | corridor, two posts r 0.10 at x = 1.0, gap 0.60 to 0.28 m | (2.0, 0.0) |
| `lab2_t3` `lab2_t4` | Lab 2 tasks, use `footprint:=burger` and `plank:=false` on the launch | topic / circle |
