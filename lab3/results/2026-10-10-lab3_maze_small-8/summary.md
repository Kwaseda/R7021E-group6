# 2026-10-10-lab3_maze_small-8

World `lab3_maze_small`. Code: commit aa65d6f (ramp fix); the stop fix f632b52 came after and does not change driving. All numbers [gazebo], one run.

Complete exploration.

| Item | Value |
|---|---|
| `exploration finished` printed | yes, 201 s after the first plan |
| goals (plan lines) | 18 |
| arrived / stalled / not reachable / path is old | 16 / 0 / 0 / 2 |
| back-offs in the follower | 0 |
| known area at the end | 15.2 m2 |
| `known` first above 90 % of the final value | 156.0 s after the first plan |
| longest / median `plan_time` | 0.09 / 0.045 s |
| driven (true pose) | 20.2 m |
| closest robot centre to a true wall | 0.107 m (Burger half width 0.089 m) |
| largest SLAM pose error against Gazebo truth | 0.16 m at 198.0 s |

Frontier clusters of 5 or more cells left at the end:

```
none
```

The bag is in `~/bags/2026-10-10-lab3_maze_small-8` and is not in git.
