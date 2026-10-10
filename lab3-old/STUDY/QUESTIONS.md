# Lab 3 questions

Answer each one out loud in two sentences, with no notes. The answers are in `THEORY.md` and
in the code. The file does not hold them on purpose.

How to use it:

1. Read one module of `THEORY.md`. Close it.
2. Answer that module's questions here. Say "I don't know" when you do not know. That is a
   good answer. Then open the file or the code and find the answer.
3. Two days later, answer the same questions again before you open anything.
4. A wrong answer comes back after 1, 3 and 7 days.

---

## T1. Map, frames and TF

1. A map is 200 cells wide. The origin is (-5.0, -5.0). The resolution is 0.05 m. Which row and
   column hold the point (0, 0)? What is its index in the flat list?
2. Why does NumPy reshape the list to `(height, width)`, and not `(width, height)`?
3. What do -1, 0 and 100 mean? Which values does `planning_mask` call free? Which does it call wall?
4. Why does the planner read the pose from TF (`map` to `base_link`) and not from `/odom`?
5. Which node publishes the transform `map` to `odom`? Which publishes `odom` to `base_link`?
6. In Gazebo you see "extrapolation" and "lookup failed" in the log. What do you check first?

## T2. Safe space (Task 3)

1. Why must the map be inflated for RRT*? What does RRT* assume about the robot?
2. How many cells is the collar? Show the calculation. Why round up?
3. What does the despeckle step remove? Why does it run before the dilation?
4. What breaks if the inflation is too large? What breaks if it is too small?
5. A corridor is 0.40 m between the wall faces. Can the robot plan through it? And at 0.30 m?
6. What does `reachable` do? Describe one run that fails without it.
7. Why does `segment_free` check the edge every half cell, and not only the new node?
8. The robot stands inside the collar. How does the tree start?
9. Why is APF a poor main planner? Which other layer in our stack protects the robot?

## T3. RRT and RRT* (Task 2)

1. List the five steps of RRT in order. Which one needs the map?
2. What does the step size trade off? What does goal bias 0 do? What does goal bias 0.9 do?
3. What does RRT* add to RRT? What does it cost?
4. Point at the lines in `rrt_star` that choose the parent. Point at the lines that rewire.
5. Why does the tree grow 200 more iterations after the first hit?
6. Why is there a second budget of 15000?
7. Why sampling, and not A* on the grid? Name one case where A* wins.
8. The rewire radius is 0.60 m and the step is 0.30 m. What happens if you set the rewire
   radius to 0.10 m? Say it before you run it.

## T4. Frontiers and information gain (Task 4)

1. Define a frontier cell the way the detector does. Which neighbours does it check?
2. How do frontier cells become goals? Name the two helper functions.
3. Write the score H. Give the unit of each term.
4. What happens when w is 0? What happens when w is very large?
5. Why do we count frontier cells in 0.75 m and not in the full 3.5 m range of the laser?
6. The lab says "count unknown cells". Our I counts frontier cells. Argue for our choice in
   two sentences. Then argue against it in two sentences.
7. Greedy or highest gain: which one finishes the first 90 % faster? Which one maps more
   completely? How do you know which one your robot is?
8. What ends the run in our code? What would you add for "a specified criterion is met"?

## T5. Following the path

1. The path has five points. The robot is on the first one. Which points does the loop remove?
2. Why does the loop never remove the last point?
3. Describe the forward speed when the distance to the target falls from 1 m to 0. Where is the
   kink? Which two parameters set it?
4. The heading error is 2.5 rad. What command goes out? What changes below 0.3 rad?
5. A wall is 0.24 m ahead, inside the strip. What fraction of the speed is left?
6. A wall is 0.15 m to the side. What does the strip check do?
7. The robot swings left and right around the path. Which parameter do you change first, and
   in which direction?
8. The robot reaches the end of the path. What does the follower do? How does the planner find out?
9. What is the blind zone of the laser? Does the robot get there?

## T6. The loop and how it fails (Task 5)

1. Draw the loop. What starts each step and what ends it?
2. Why does the planner run on a 1 s timer and not in the map callback?
3. Name four ways the loop can fail. For each one, say what the robot does now.
4. The log says `stalled, crossing off the goal` twice in a row. What do you look at?
5. The map changes while RRT* runs. What does that do to the path?
6. The run ends with `exploration finished` but the RViz map still shows frontiers. Give two
   reasons.

## The node map

1. Draw from memory every node and every topic between `/scan` and `/cmd_vel`. Mark the
   node that you wrote.
2. Which node decides where to go? Which decides how to get there? Which decides how to drive?
3. `navigation_node` stops while the robot is driving. What do you see in RViz? What does the
   robot do?
4. `path_follower_node` stops while the robot is driving. What do you predict? Check it in
   Gazebo, then use the stop command.
5. The message type of `/cmd_vel` is `TwistStamped`. What happens if one side expects `Twist`?
6. A node subscribes to a topic that nobody publishes. What error do you see?
7. The same node has two names: one with `ros2 run` and one with `ros2 launch`. Which two nodes
   do this? Which name do you type in `ros2 param get`?

---

## Change it live

At the oral, the teacher can ask for a change. For each row: say what will happen in RViz, in
the log and in the robot, then make the change, run, and compare. Write the prediction down first.

| Change | File and name | Say first |
|---|---|---|
| No inflation | `navigation_node.py`, `inflation_radius` 0.105 to 0.0 | Path, walls, stalls |
| Huge inflation | `inflation_radius` to 0.30 | Which goals vanish |
| Greedy goals | `info_weight` 0.10 to 0.0 | Order of goals, time to 90 % |
| Info only | `info_weight` to 1.0 | Order of goals, distance driven |
| Small step | `step_size` 0.30 to 0.10 | Tree shape, plan time, plan failures |
| No goal bias | `goal_bias` 0.10 to 0.0 | Tree shape, plan time |
| Weak search | `max_iterations` 1500 to 100 and `retry_iterations` to 100 | Which goals fail |
| Small rewire radius | `rewire_radius` 0.60 to 0.10 | Why |
| Impatient stall rule | `stall_timeout` 8.0 to 1.0 | Which goals get crossed off |
| Sparse path | `waypoint_spacing` 0.10 to 0.50 | Corners, cuts |
| Faster robot | `path_follower_node.py`, `max_v` 0.15 to 0.22 | Corners, scan slow-down |
| Twitchy turns | `kp_yaw` 2.0 to 6.0 | Path tracking |
| Long look-ahead | `look_ahead` 0.2 to 0.6 | Corners |

After each run, put the value back. `git diff` must be empty.

---

## Decisions you defend

Write your two sentences in the last column, from memory, after the module. Write them again
the day before the oral.

| Decision | The alternative | Your defence |
|---|---|---|
| RRT* | RRT, or A* on the grid | |
| Frontier goals | Next-best view inside the tree | |
| Map inflation | APF, or a volumetric check | |
| I counts frontier cells | I counts unknown cells | |
| Radius 0.75 m for I | The full laser range | |
| A 1 s timer plans | Planning in the map callback | |
| Pose from TF | Pose from `/odom` | |
| Follower kept, three additions | Rewrite the follower | |
| One navigation file | Split into modules | |
| Loop closure on | Loop closure off | |
| Cross off goals for good | Allow them again later | |

## The four report questions

Answer each one in a short paragraph. The lab PDF asks for these.

1. Your ROS 2 node structure: inputs, outputs, workflow.
2. How the planner works, with pseudocode.
3. The exploration method and the equation for the information gain.
4. The problems you met in the real-time implementation.
