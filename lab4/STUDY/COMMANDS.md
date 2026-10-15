# Lab 4 commands

These commands ran on this laptop. For the hall, use `../RUNSHEET.md`; it has the order and the
checks. Robot number X: turtle X, IP `192.168.50.X0`, domain `3X`.

Run every command from the package folder, so `runs/` and `figures/` are the right folders:

```bash
cd ~/R7021E-group6/lab4/ros2_ws/src/r7021e_fast_slam
```

## Every terminal

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
source ~/R7021E-group6/lab4/ros2_ws/install/setup.bash
```

## Build

```bash
cd ~/R7021E-group6/lab4/ros2_ws && colcon build --packages-select r7021e_fast_slam --symlink-install
```

With `--symlink-install`, a change to a `.py` file or to `params.yaml` needs a node restart, not
a build.

## Stop the robot

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{}"
```

## Gazebo

Kill an old simulator first. Ctrl-C does not stop the Gazebo server:

```bash
pkill -f "gz sim"
```

```bash
ros2 launch ~/R7021E-group6/lab3/sim/maze_world.launch.py world:=lab3_maze_small
```

## The filter

Live in Gazebo:

```bash
ros2 launch r7021e_fast_slam r7021e_fast_slam.launch.py use_sim_time:=true num_particles:=10 run_name:=test
```

On the robot (no `use_sim_time`):

```bash
ros2 launch r7021e_fast_slam r7021e_fast_slam.launch.py num_particles:=10 run_name:=hall_run1
```

Only `num_particles`, `seed`, `use_improved_proposal` and `run_name` can be set on the command
line. Everything else is in `config/params.yaml`. The node writes `runs/<run_name>.npz` and
`.json` on Ctrl-C.

## Bags

Record (start before the robot moves):

```bash
ros2 bag record /scan /odom /tf /tf_static -o runs/hall_run1
```

Check:

```bash
ros2 bag info runs/hall_run1
```

Replay through the filter (two terminals):

```bash
ros2 launch r7021e_fast_slam r7021e_fast_slam.launch.py use_sim_time:=true run_name:=replay_test
```

```bash
ros2 bag play runs/maze_drive --clock
```

## The Task 7 sweep (about 2 h)

```bash
bash tools/run_sweep.sh runs/maze_drive "1 5 10 20 50" "1 2 3"
```

## The four figures

```bash
python3 -m r7021e_fast_slam.plot_results --figure maps --runs runs/mapping_tuned.npz runs/fastSLAM2.npz --out figures
```

```bash
python3 -m r7021e_fast_slam.plot_results --figure ate --runs runs/fs*.npz --out figures
```

```bash
python3 -m r7021e_fast_slam.plot_results --figure neff --runs runs/neff_selective.npz runs/neff_every.npz --out figures
```

```bash
python3 -m r7021e_fast_slam.plot_results --figure timing --runs runs/fs2_n*_s1.npz --out figures
```

## Checks when something is silent

```bash
printenv ROS_DOMAIN_ID
```

```bash
ros2 topic info /cmd_vel -v
```

```bash
ps aux | grep "[g]z sim"
```

```bash
ros2 topic hz /scan
```
