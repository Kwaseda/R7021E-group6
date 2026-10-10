# 2026-10-10-lab3_maze-4

World `lab3_maze`. Code: commit aa65d6f (ramp fix); the stop fix f632b52 came after and does not change driving. All numbers [gazebo], one run.

Complete exploration of the big maze.

| Item | Value |
|---|---|
| `exploration finished` printed | yes, 1325 s after the first plan |
| goals (plan lines) | 98 |
| arrived / stalled / not reachable / path is old | 58 / 3 / 0 / 37 |
| back-offs in the follower | 9 |
| known area at the end | 49.1 m2 |
| `known` first above 90 % of the final value | 1174.0 s after the first plan |
| longest / median `plan_time` | 2.69 / 0.14 s |
| driven (true pose) | 132.8 m |
| closest robot centre to a true wall | 0.104 m (Burger half width 0.089 m) |
| largest SLAM pose error against Gazebo truth | 0.34 m at 1323.0 s |

Frontier clusters of 5 or more cells left at the end:

```
none
```

The bag is in `~/bags/2026-10-10-lab3_maze-4` and is not in git.
