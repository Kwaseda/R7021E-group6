# 2026-10-10-lab3_maze_small-10

World `lab3_maze_small`. Code: f632b52 + trail-as-free fix (not committed). All numbers [gazebo], one run.

Failed. The follower blocked on a wall parallel to the robot at the edge of the strip, and the frontier around the bottom-left corner fell inside the retire radius of an arrived goal.

| Item | Value |
|---|---|
| `exploration finished` printed | yes, 197 s after the first plan |
| goals (plan lines) | 11 |
| arrived / stalled / not reachable / path is old | 4 / 3 / 0 / 4 |
| back-offs in the follower | 24 |
| known area at the end | 4.3 m2 |
| `known` first above 90 % of the final value | 30.0 s after the first plan |
| longest / median `plan_time` | 0.08 / 0.03 s |
| driven (true pose) | 8.2 m |
| closest robot centre to a true wall | 0.12 m (Burger half width 0.089 m) |
| largest SLAM pose error against Gazebo truth | 0.06 m at 38.0 s |

Frontier clusters of 5 or more cells left at the end:

```
size  14 mid (-0.38,  1.59)  retired near (-0.38, 1.59)
size  14 mid (-1.21, -1.56)  retired near (-1.44, -1.28)
```

The bag is in `~/bags/2026-10-10-lab3_maze_small-10` and is not in git.
