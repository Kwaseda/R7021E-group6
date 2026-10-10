# 2026-10-11-nbv-small-2

World `lab3_maze_small`. Code: NBV planner. All numbers [gazebo], one run.

Full exploration. exploration finished printed 29 s after the last arrival (the summary script looks for the old wording).

| Item | Value |
|---|---|
| `exploration finished` printed | no |
| goals (plan lines) | 56 |
| arrived / stalled / not reachable / path is old | 54 / 0 / 0 / 2 |
| back-offs in the follower | 0 |
| known area at the end | 15.5 m2 |
| `known` first above 90 % of the final value | 168.0 s after the first plan |
| longest / median `plan_time` | 0.26 / 0.22 s |
| driven (true pose) | 23.6 m |
| closest robot centre to a true wall | 0.14 m (Burger half width 0.089 m) |
| largest SLAM pose error against Gazebo truth | 0.22 m at 195.0 s |

Frontier clusters of 5 or more cells left at the end:

```
none
```

The bag is in `~/bags/2026-10-11-nbv-small-2` and is not in git.
