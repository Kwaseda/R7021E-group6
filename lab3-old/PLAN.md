# Lab 3 plan: autonomous exploration with RRT

Week of 2026-10-06. Present when the Gazebo runs are solid, not before.

## What you actually write

Two holes in one file, `r7021e_exploration/navigation_node.py`:

```python
# self.path_pub =  ...          <- hole 1, commented out in the template
def plan_path(self, start, map_msg, frontier_msg):
    return None                 <- hole 2
```

That is the deliverable. Everything else in the lab is provided.

## What is given, and must not be rewritten

| file | what it does | interface |
|---|---|---|
| `frontier_detector_node.py` | finds known/unknown boundary cells | `/map` -> `/frontiers`, frontier cells = 100 |
| `path_follower_node.py` | drives the robot along a path | `/path` -> `/cmd_vel` |
| `navigation_node.py` | template, TF lookup and callbacks already written | `/map`, `/frontiers` -> `/path` |
| `exploration.launch.py` | starts all three plus slam_toolbox and RViz | |
| `slam_config.yaml` | slam_toolbox settings | |

The template already hands you `get_robot_pose()` returning `(x, y, yaw)` in the map frame,
`quat_to_yaw`, `yaw_to_quaternion`, and an `_on_map` callback that calls `plan_path` every time
the map updates. Read it before writing anything.

`path_follower` defaults: `max_v` 0.15, `max_w` 1.0, `kp_vel` 1.0, `kp_yaw` 2.0,
`look_ahead` 0.2. You tune these, you do not rewrite the node.

### One bug in the template, find it before it costs you an hour

The template declares `frontier_topic` defaulting to `'frontier'`. The detector publishes
`'frontiers'`. Subscribing to the wrong name fails silently, same as every other ROS mismatch.

## What to delete from last time

The old Lab 3 was two packages, seven modules, three YAML files, two launch files and five test
files. It also contained a reimplemented `path_follower`, which the course already gives you.
None of it carries over. Start from the Canvas package, unmodified, and fill in the one file.

The two Gazebo maze worlds are worth keeping, because they are assets rather than architecture:
`lab3_maze_small.world` is 4 x 4 m for fast iteration, `lab3_maze.world` is 7.2 x 7.2 m for the
final run.

## Theory you will be asked about

Be able to answer each of these in two sentences, out loud, without notes.

**Occupancy grid.** `-1` unknown, `0` free, `100` occupied. `info.resolution` metres per cell,
`info.origin` is the world position of cell (0,0). World to grid is
`col = (x - origin.x) / resolution`. Getting this backwards is the classic lab 3 bug.

**Why sampling-based planning.** A grid search scales with the number of cells; RRT samples the
space and so copes with large or poorly known maps. It is probabilistically complete, not
optimal. RRT* adds rewiring and converges toward optimal as samples grow.

**RRT, the five steps.** Sample a random point, find the nearest node in the tree, steer from it
toward the sample by a fixed step, check the segment for collision, add the node. RRT* then
rewires neighbours if going through the new node is cheaper.

**Frontier.** A cell that is free and adjacent to unknown. The boundary of what you have
mapped, so it is where new information can be obtained.

**Information gain.** The number of currently unknown cells a viewpoint would reveal. The lab's
tip matters: use a much shorter LiDAR range for the gain computation than the real sensor, or
every frontier looks equally good and the robot dithers.

**Choosing the next goal.** Utility = gain minus a weight times travel cost. Pure nearest
frontier is greedy and fast; pure maximum gain wanders. The weight is the knob, and the report
asks you to justify it.

**Why TF and not /odom.** SLAM corrects the robot's pose as it closes loops, so the map frame
and the odom frame drift apart. The template looks up `map -> base_link` for that reason.

## Days

**Day 1, setup and Task 1.** Fresh `~/ros2_ws`, extract the Canvas package, `colcon build`,
`sudo apt install ros-jazzy-slam-toolbox`. Launch it in Gazebo and watch the map build under
teleop. Then Task 1: publish a hardcoded two-point `Path` and watch the robot drive it. No
planning yet. This proves the interface and is the whole of Task 1.

**Day 2, RRT at a desk.** Write the planner as plain functions above the node class, no ROS
imports: `world_to_grid`, `grid_to_world`, `is_free`, `collision_free`, `rrt`. Add a
`--selftest` that runs them against a synthetic grid and prints whether a path was found. You
should be able to test the planner without Gazebo running at all. This is also the pseudocode
the report asks for.

**Day 3, Task 2 in the maze.** Wire `plan_path` to call `rrt` with a hardcoded goal, publish
the `Path`, watch it drive. Visualise the tree as a `MarkerArray` while debugging; it is the
difference between seeing the bug and guessing. Use the small maze.

**Day 4, Tasks 3 and 4.** Inflate the map by the robot radius before collision checking, about
ten lines, because RRT treats the robot as a point and the instructions warn about exactly
this. Then the information gain: for each frontier cell, count unknown cells within a reduced
sensor range, score `gain - w * distance`, pick the best.

**Day 5, Task 5 and recording.** Close the loop: pick goal, plan, publish, wait for arrival,
repeat until no frontiers remain. Run the big maze. Record a screen capture of RViz and a bag.
Note the time to 90 percent and to full coverage, since the optional competition uses both.

## Gazebo, two terminals, no new files

```bash
export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

```bash
ros2 launch r7021e_exploration exploration.launch.py
```

Swap in a maze world by pointing `GZ_SIM_RESOURCE_PATH` at the worlds folder. Do not write a
new launch file for simulation; the course one already starts SLAM, RViz and all three nodes.

## The report

Around three pages with figures, focused on Task 4. It has to answer four things, so collect
them as you go rather than at the end:

1. The ROS 2 node structure, inputs, outputs, workflow.
2. How the planner works, with pseudocode.
3. Which exploration method, with the information gain equations.
4. Problems hit during real-time implementation.

Question 4 is the one that caught you out in Lab 2. Keep a running note of every failure and
what it turned out to be, the same way `lab2/report_notes.md` ended up being written.

## Working method

Agreed from the Lab 2 retrospective: stub every function signature first, then fill them in one
at a time. One function per sitting, each time saying out loud what calls it and what it returns
to. Do not accept a finished file. The reason Lab 2 was hard to defend is that the parts arrived
assembled.
