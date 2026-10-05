# Lab 2 run sheet

Robot: turtle ___   IP 192.168.50.__0   domain 3__
Mapping: turtleN -> 192.168.50.N0 -> domain 3N

## Stop the robot

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{}"
```

Keep that pasted in a spare terminal. The node also publishes zeros on Ctrl-C.

## Every terminal starts with these two

```bash
export ROS_DOMAIN_ID=3X
source ~/R7021E-group6/lab2/ros2_ws/install/setup.bash
```

Miss the export and the terminal sees nothing, with no error. Miss the source and you get
"Package 'r7021e_lab2' not found". Check any time with `printenv ROS_DOMAIN_ID`.

## Build

Once per session, from the workspace root:

```bash
cd ~/R7021E-group6/lab2/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

Because of `--symlink-install`, editing `mpc_controller.py` after this needs no rebuild. Just
restart the node. Changing `setup.py` or `package.xml` does need one.

---

# Running the tasks

## Simulation

```bash
ros2 launch r7021e_lab2 lab2_launch.py task:=1 sim:=true
```

## Real robot

SSH terminal, on the robot, no export:

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

Your laptop:

```bash
ros2 launch r7021e_lab2 lab2_launch.py task:=1 domain_id:=3X
```

## Task 1, setpoint tracking

Bounds +/-1.0, no obstacles. Inside the box first:

```bash
ros2 topic pub --once /new_position geometry_msgs/msg/Pose "{position: {x: 0.8, y: 0.5}}"
```

Then outside it, which is what 1b actually asks for:

```bash
ros2 topic pub --once /new_position geometry_msgs/msg/Pose "{position: {x: 1.5, y: 0.0}}"
```

The robot stops at x = 1.0 and logs the solver status. That is the right answer. The goal sits
outside the feasible set the boundary defines, so the boundary is the closest point it is
allowed to reach.

## Task 1c, varying t_step and horizon

```bash
ros2 launch r7021e_lab2 lab2_launch.py task:=1 sim:=true t_step:=0.2 n_horizon:=10
```

Try 0.1/20, 0.2/10, 0.05/40. The number that changes behaviour is neither one alone, it is
`n_horizon * t_step * v_max`. At 20, 0.1 and 0.22 that is 0.44 m of reach. The node prints it
on startup, so read that line.

## Tasks 2 and 3, obstacles

```bash
ros2 launch r7021e_lab2 lab2_launch.py task:=2 sim:=true
```

```bash
ros2 topic pub --once /new_position geometry_msgs/msg/Pose "{position: {x: 1.8, y: 0.0}}"
```

Same for `task:=3`, same goal. Task 2 is one obstacle at (0.75, 0.08) r 0.30. Task 3 is a
slalom, (0.70, 0.15) and (1.30, -0.15), both r 0.15.

## Task 4, circle with an obstacle

```bash
ros2 launch r7021e_lab2 lab2_launch.py task:=4 sim:=true
```

Publish nothing. It waits 8 s, then drives two 60 s laps of a 0.8 m circle on its own. The
obstacle sits at (0.00, 0.62), inside the circle, and this is the only task using a soft
constraint.

---

# Recording and plotting

```bash
cd ~/R7021E-group6/lab2/bags
ros2 bag record -o task1 /odom /cmd_vel /scan /new_position /mpc_prediction /tf /tf_static
```

Start the recorder before the node. Ctrl-C the node first, then the recorder. Press Ctrl-C
once and wait, the recorder keeps writing for a while after.

```bash
ros2 bag info task1
```

```bash
cd ~/R7021E-group6/lab2/plots && python3 plot_rosbag.py ../bags/task1 --task 1
```

That prints path length, duration, final position, closest approach to each obstacle against
the constraint radius, and circle tracking error for task 4. Add `--animate` for a GIF.

---

# What --once, --times and --rate actually do

This is the question you got asked and could not answer.

- `--once` publishes one message and exits.
- `--times N` publishes N messages and exits.
- `--rate N` is the publishing frequency in Hz. Default 1. It only means anything if you are
  sending more than one message.
- `-w N` waits until N subscribers have matched before publishing.

The part that matters: **`-w` defaults to 1 when you use `--once` or `--times`.** So `--once`
already waits for one subscriber to connect before it sends anything. The controller is always
that subscriber. One publish is enough, and your teacher was right.

Check it yourself:

```bash
ros2 topic pub --help | grep -A4 wait-matching
```

So why did the old notes say publish six times? Insurance against a race that `-w 1` does not
cover. `-w 1` is satisfied by the controller alone, so a bag recorder that is still finishing
discovery can miss the message. The robot moves, the goal never lands in the recording, and the
plot comes out with no goal on it.

The clean fix is to say how many subscribers you want, not to spam:

```bash
ros2 topic pub --once -w 2 /new_position geometry_msgs/msg/Pose "{position: {x: 1.8, y: 0.0}}"
```

Two subscribers: the controller and the recorder. Use plain `--once` when you are not recording.

---

# Launch arguments

| argument | values | default |
|---|---|---|
| `task` | 1, 2, 3, 4 | 1 |
| `sim` | true, false | false |
| `rviz` | true, false | true |
| `t_step` | seconds | 0.1 |
| `n_horizon` | steps | 20 |
| `domain_id` | 31 to 39 | 32 |

Anything else is a node parameter, so use `ros2 run` for it:

```bash
ros2 run r7021e_lab2 mpc_node --ros-args -p task:=2 -p max_linear_velocity:=0.15
```

Available: `max_linear_velocity`, `max_angular_velocity`, `min_linear_velocity`, `q_position`,
`q_terminal`, `r_input`, `goal_tolerance`, `odom_timeout`, `obstacle_penalty`.

Note `ros2 launch` rejects `--ros-args`, and `ros2 run` ignores `name:=value`. Different syntax,
easy to mix up under pressure.

---

# Changing geometry mid-session

Obstacles and boundaries live in the `TASKS` dict at the top of `mpc_controller.py`. Edit, save,
restart the node. No rebuild.

```python
TASKS = {
    1: dict(bound=1.0, obstacles=[],                                        soft=False, goal='topic'),
    2: dict(bound=2.0, obstacles=[(0.75, 0.08, 0.30)],                      soft=False, goal='topic'),
    3: dict(bound=2.0, obstacles=[(0.70, 0.15, 0.15), (1.30, -0.15, 0.15)], soft=False, goal='topic'),
    4: dict(bound=1.5, obstacles=[(0.00, 0.62, 0.12)],                      soft=True,  goal='circle'),
}
```

Each obstacle is `(x, y, radius)` before inflation. Radii get `+0.105` at build time for the
robot's own footprint.

Before you move anything:

1. Never put an obstacle exactly on the straight line from start to goal. Left and right detours
   then cost the same and the solver has no gradient to break the tie. The robot drives up to
   the keep-out boundary and stops. Offset y by 0.08 and it works.
2. The detour has to fit inside 0.44 m of reach. A big obstacle, or one far off the path, asks
   for more swing than the horizon can see, and the problem goes infeasible.
3. For task 4 the obstacle must sit inside the circle. Outside needs a 0.37 m excursion, which
   is more than the reach. Centred on the circle puts the reference itself inside the keep-out
   zone and the solver's restoration fails.

Want a task that was not asked for? Copy an entry, give it key 5, and run `task:=5`.

---

# Troubleshooting

| symptom | cause | fix |
|---|---|---|
| `ros2 topic list` nearly empty | wrong domain in that terminal | `printenv ROS_DOMAIN_ID`, export it |
| "Package 'r7021e_lab2' not found" | not sourced | `source install/setup.bash` |
| `solver failed: Infeasible_Problem_Detected` | goal outside the bounds, or an obstacle blocking with no room to detour | pick a nearer goal, or widen `bound`, or shrink the obstacle |
| Robot stops dead at an obstacle and never recovers | a measurement landed inside the hard keep-out zone. Step zero of the horizon is the measurement, not a decision variable, so no input can fix it | set `soft=True` for that task |
| Robot never moves, topics look fine | message type mismatch | `ros2 topic info /cmd_vel -v`, both ends must say TwistStamped |
| Robot never moves on hardware | motors off | `ros2 service call /motor_power std_srvs/srv/SetBool "{data: true}"` |
| Goal published, nothing happens | node did not log NEW GOAL, so it never arrived | check the domain in both terminals |
| RViz blank, red global status | fixed frame | set it to `odom` |
| RViz shows the robot but no laser dots | QoS | LaserScan reliability to Best Effort |
| No orange prediction line in RViz | solver is failing, or `/mpc_prediction` not published | read the node's log |
| Solve slower than `t_step` | horizon too long | drop `n_horizon` to 15, or raise `t_step` |
| Bag folder empty | wrong domain, or recorder stopped before the node | `rm -rf` it and redo, recorder first |
| Robot teleports in RViz, TF_OLD_DATA | stray `gz sim` from a previous run | `ps aux \| grep "gz sim"` and kill it |
| Simulation launches twice over | leftover launch process | same as above |

## Quick checks

```bash
ros2 node list
ros2 topic info /cmd_vel -v
ros2 topic echo /odom --field pose.pose.position
ros2 param get /mpc_node task
ps aux | grep -E "gz sim|rviz2" | grep -v grep
```

---

# If he asks

**Why no trajectory node for task 4?** The controller generates the reference itself, one point
per horizon step, `circle_point(elapsed + k*t_step)`. The horizon sees where the circle is going,
not just where it is now. A separate node publishing a path would be two processes and a topic
doing what ten lines do locally.

**Why 0.22 and not the tutorial's 0.8?** That is the Burger's actual limit. It also changes the
reach from 1.6 m to 0.44 m, which is why the tutorial's own obstacle coordinates go infeasible
on this robot. The lab sheet says to adjust the constraints if necessary. This is the number
behind that.

**Why is task 4 soft when 2 and 3 are hard?** A hard constraint is enforced at every horizon
step including step zero, and step zero is the measurement. Tasks 2 and 3 pass their obstacles
with margin so a millimetre of overshoot changes nothing. Task 4 rides the constraint boundary,
clearing by about 0.1 mm with no actuation delay, so any latency pushes the measurement inside
and the problem is infeasible forever. The soft constraint behaves identically whenever the hard
one is satisfiable, and the slack is spent against the 105 mm inflation margin, not real
clearance.

**Why n_robust=0?** No uncertain parameters in the model, so anything higher just makes the
problem bigger for nothing.

**Why squared distances in the obstacle constraint?** `r^2 - d^2 <= 0` is the same condition as
`d >= r`, with no square root for the solver to differentiate.
