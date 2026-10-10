# Lab 3 report notes

Notes for the report we write after the lab. Gazebo numbers come from `../lab3-old/results/`
and `results/`. The robot section is empty until Tuesday.

## Where we started

Our lab plan was frontier-based. The frontier detector gives a grid of frontier cells. We
grouped them into 8-connected clusters, took the middle of each cluster (or the nearest free
cell if the middle was inside a wall's padding) as a candidate goal, and ran a goal-directed
RRT* to the six nearest candidates. Each path got a score, H = L - w*I + c*turn, and the lowest
one won. That version lives in `lab3-old`.

It worked in the end. In the 4 m Gazebo maze it explored everything in 4 runs out of 4, and in
the 7.2 m maze it got one full run and one at about 85 %. But getting there took a lot of
machinery. Clusters, a flood fill to check that a goal connected to the robot, a list of
crossed-off goals, a retire radius around each one, a cap on candidates, a second RRT* budget
for long paths, retry rounds for stalled goals. The navigation node grew to 638 lines with 29
tunable values. Most of those parts were patches for one failure each, and by the end we had
trouble explaining why some of them were needed at all.

## Why we changed to next-best-view

The lab PDF describes two options for Task 4. For next-best-view it says to evaluate the
information gain at the edge nodes of the RRT. When we read that again, it was clear that the
tree we were already growing could give us the candidates directly. So `lab3` does exactly
that: grow one RRT* tree from the robot over the inflated free space, score every leaf, drive
the branch to the best one.

What we gained:

- The planner got a lot smaller. 330 lines and 13 values instead of 638 and 29. Clustering,
  the flood fill, candidate selection and the per-goal RRT* runs are all gone.
- Reachability comes for free. A node only joins the tree through a collision-free edge, so
  every leaf is a place the robot can get to. In `lab3-old` we needed a separate flood fill for
  this. Without it, 5 of 12 desk runs stopped at 57 % because the robot kept aiming at a sliver
  of free space it could not reach.
- Sampling only free cells means no sample lands in a wall or in unknown space, so the tree
  never plans into the unknown.
- It explored the small maze faster. 90 % of the final map came at 168 s, against roughly 190
  to 260 s for the old planner on the same maze. Zero stalls and zero back-offs in that run,
  and no plan took longer than 0.26 s. That is one run, so we would not lean on the exact
  number, but the robot clearly wasted less time.
- The score is easier to explain. One formula on one set of candidates, and the candidates are
  just the tips of the tree.

## What it cost

The big maze showed the weak spot. Information gain counts frontier cells within 0.75 m of a
leaf, including cells behind a wall. Some of those cells never clear, because no free spot can
see them. The first NBV version kept driving toward them in 0.2 m steps, so we added a rule: a
frontier cell that is still there after the robot has stood within 0.5 m of it stops counting.
That fixed the small maze.

In the 7.2 m maze the first NBV run still failed: 14.5 of about 52 m2 after 30 minutes, with 74
stalls. Leaves the follower could not reach kept getting picked again. We then also marked the
goal itself as seen when the robot stalled. The second run (cut short at 1124 s when the laptop
rebooted) reached 38.8 m2 and was still growing, but the map sat at 15.1 m2 for about eight and a
half minutes in the middle of it. The old planner did better in that maze. So for mazes around
4 to 5 m, NBV is the better choice. For bigger ones we are not sure yet.

## Things we kept from the old version

These came out of the Gazebo runs on 2026-10-10 and apply to both planners:

- Inflation of 0.105 m (the Burger radius, rounded up to 3 cells) for Task 3.
- A laser stop in the path follower. With it turned off, the big-maze run hit a wall when the
  SLAM pose drifted 2.6 m. Inflation only protects you while the pose is right.
- A 0.10 m back-off when the follower is blocked for 2 s, otherwise it froze at corners.
- `look_ahead` 0.12 m instead of 0.20, because 0.20 made it aim past corners into walls.
- Slower turning (`kp_yaw` 1.0, `max_w` 0.6). Gazebo odometry reported 41 % more rotation than
  actually happened when the robot turned fast while driving.
- Loop closure off. Twice it closed the loop on the wrong corridor and the pose error jumped
  from 0.15 m to 1.7 m in four seconds.
- A zero velocity command on Ctrl-C. `turtlebot3_node` keeps the last command it got, and the
  default rclpy signal handler shuts the context down before the stop can be sent. Before the
  fix, Ctrl-C left the robot driving at 0.15 m/s.

## Problems from the real-time implementation (report section 4)

Candidates, each with a run behind it: the follower freezing at corners; the slow-down ramp that
never reaches the stop distance; odometry slip; false loop closures; doors that close behind the
robot once the padding is applied (the robot got into a 0.40 m room and the planner then thought
it was boxed in); frontier behind walls; the bag recorder ignoring SIGINT in scripts; leftover
`gz sim` processes after Ctrl-C. From the Lab 1 robot bags: the robot clock was 325 days behind
the laptop, and laptop receive gaps reached 3.6 s on bad Wi-Fi.

## Still open, to fill in after Tuesday

- How the robot run went: time to 90 % and to 100 %, video link, RViz recording.
- Whether real odometry slips less than Gazebo's, and whether the turning limits still matter.
- Task 4 comparison of `info_weight` 0.0, 0.10 and 0.30 (greedy vs complete).
- Narrowest passage in the hall maze, and what the robot did there.
