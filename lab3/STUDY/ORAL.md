# Lab 3 oral

The oral on Monday 12 Oct has two parts: a talk and a demo. Use this file to prepare both. It
holds headings, an order of work and a failure script. It does not hold the sentences you say.
You write those.

---

## The opening: five sentences

Write one sentence under each heading. Do it on Sunday evening, on paper, from memory. Do not
look at `THEORY.md`. Then say all five out loud, two times. Then check each sentence against the code.

1. The node map. Which nodes? Which topics join them? Which node did you write?
2. Task 2. What does the planner do when the goal is known?
3. Task 3. What keeps the robot away from the walls?
4. The score H. Name each term. Say why w is 0.10.
5. One weak point. Choose one from "Known weak points" in `LOG.md`, or from the last list in
   `CODEMAP.md`. Say what you would do about it.

A good sentence holds a name or a number from the code. A sentence with neither is too vague.
The teacher can ask about any word in it, so write only what you can defend.

While you talk, draw the node map on paper. Draw it from memory.

---

## The demo, in order

1. Open a spare terminal. Paste the stop command. Do not press Enter.

   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{}"
   ```

2. In each terminal, run the three start lines. They are in `COMMANDS.md`, under "Every terminal
   starts with these lines".
3. In Gazebo only: `ps aux | grep -E "[g]z sim"` prints nothing.
4. Start the screen recording. Start the launch.
5. `ros2 node list` shows each node one time.
6. Say your opening while the robot explores. Use the table below to point at RViz.
7. At `exploration finished`, or at your own time limit, show the whole map in RViz.
8. Say what you saw in one sentence. Name one thing that looked weak.

## Three layers and the fail-over rule

| Layer | What | How to start it |
|---|---|---|
| 1 | The real robot | The hall procedure in `COMMANDS.md`. Use it only if you have robot time. No `use_sim_time`. Compare the clocks first |
| 2 | Gazebo on your laptop, `lab3_maze_small` | The two commands below |
| 3 | The Saturday recording of a full run in `lab3_maze` | Open the video and the launch log |

```bash
ros2 launch ~/R7021E-group6/lab3/sim/maze_world.launch.py world:=lab3_maze_small
ros2 launch r7021e_exploration exploration.launch.py use_sim_time:=true
```

These two lines come from `COMMANDS.md`. They have not run yet. When `RUNSHEET.md` exists, copy the
lines from there.

The rule: when a live run fails, you have 2 minutes to find the cause. Look at the clock. When
the 2 minutes end, say "I move to layer 2" and move. Do not go back up. Before you move, say in
one sentence what failed and what you checked.

File name of the layer 3 recording (write it on Saturday): ______________________

---

## What RViz shows, and what to say

| You see | What it is | Topic | You can say |
|---|---|---|---|
| Grey and black map | The map from slam_toolbox. Black is wall. Check the other colours in the first Gazebo run | `/map` | "The map grows when the robot sees more." |
| Green line | The path to the goal, one point each 0.10 m | `/path` | "The planner publishes the path. The follower drives it." |
| Teal lines | The RRT\* tree that found the winning path | `/rrt_tree` | "RRT\* grows this tree in free space and rewires it for a shorter route." |
| Red sphere | The goal with the lowest H | `/goal_marker` | "This goal won the score." |
| Axes on the robot | The robot frame, `base_link` | TF | "The pose comes from TF, `map` to `base_link`." |

Read one planner line out loud and name each field:

```
t=84.2 goal=(1.25, -0.80) H=2.31 L=2.05 I=18 turn=0.40 candidates=4 known=9.4m2 plan_time=0.31s
```

That line has made-up numbers. Use a real line from the run. H = L - w·I + c·turn. L is the path
length. I counts the frontier cells near the goal. The lowest H wins. The field list is in
`COMMANDS.md`, under "What the planner prints".

---

## Questions the teacher can ask

| The teacher asks about | Practise with |
|---|---|
| Your node structure | "The node map" in `QUESTIONS.md`, report question 1 |
| How the planner works | T2 and T3, report question 2 |
| How the robot explores | T4, report question 3 |
| Why you chose X and not Y | The decisions table in `QUESTIONS.md` |
| What happens if you change a value | "Change it live" in `QUESTIONS.md` |
| What went wrong in real time | T6, report question 4 |

When the teacher asks for a change, do it in this order:

1. Say what you expect to see in RViz, in the log and on the robot.
2. Edit the value in `DEFAULTS`. Restart the launch.
3. Compare with what you said.
4. Put the value back. `git diff` must be empty.

Say which numbers come from Gazebo and which come from the desk simulation. Never show a desk
number as a result.

---

## When something fails, say what you would check

A failure in front of the teacher is not the end. Naming the check out loud is part of the answer.
Use these steps:

1. Say what you see. Example: "The robot does not move."
2. Say your first check and type it. Example: `ros2 topic info /cmd_vel -v`.
3. Say what the output means.
4. Make one fix, or move down one layer when the 2 minutes end.

The table "When it breaks" in `COMMANDS.md` lists symptom, check and fix. Learn the first
five rows. When `RUNSHEET.md` exists, its table wins.

"I do not know, and I would check X with command Y" is a good answer.
