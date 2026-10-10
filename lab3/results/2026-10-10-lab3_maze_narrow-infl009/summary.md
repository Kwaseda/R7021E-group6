# 2026-10-10-lab3_maze_narrow-infl009

World `lab3_maze_narrow`. Code: commit f632b52, inflation_radius 0.09 (set back after the run). All numbers [gazebo], one run.

The robot drove through the 0.30 m door into the bottom room and was trapped the same way. Closest centre to wall 0.094 m against a half width of 0.089 m: 0.09 leaves almost no margin.

| Item | Value |
|---|---|
| `exploration finished` printed | yes, 75 s after the first plan |
| goals (plan lines) | 5 |
| arrived / stalled / not reachable / path is old | 3 / 0 / 0 / 2 |
| back-offs in the follower | 1 |
| known area at the end | 8.1 m2 |
| `known` first above 90 % of the final value | 8.0 s after the first plan |
| longest / median `plan_time` | 0.37 / 0.19 s |
| driven (true pose) | 4.7 m |
| closest robot centre to a true wall | 0.094 m (Burger half width 0.089 m) |
| largest SLAM pose error against Gazebo truth | 0.09 m at 52.0 s |

Frontier clusters of 5 or more cells left at the end:

```
size  18 mid ( 1.17,  1.19)  no reachable free cell (free but not connected)
size  17 mid (-1.43, -0.47)  no reachable free cell (free but not connected)
size  12 mid (-0.13,  1.27)  no reachable free cell (free but not connected)
size   6 mid ( 1.00,  1.42)  no reachable free cell (all in padding or not free)
```

The bag is in `~/bags/2026-10-10-lab3_maze_narrow-infl009` and is not in git.
