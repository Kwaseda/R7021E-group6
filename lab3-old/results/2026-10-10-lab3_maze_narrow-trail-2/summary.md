# 2026-10-10-lab3_maze_narrow-trail-2

World `lab3_maze_narrow`. Code: f632b52 + trail-as-free fix (not committed). All numbers [gazebo], one run.

Partial exploration.

| Item | Value |
|---|---|
| `exploration finished` printed | yes, 87 s after the first plan |
| goals (plan lines) | 5 |
| arrived / stalled / not reachable / path is old | 3 / 0 / 0 / 2 |
| back-offs in the follower | 0 |
| known area at the end | 11.2 m2 |
| `known` first above 90 % of the final value | 47.0 s after the first plan |
| longest / median `plan_time` | 0.13 / 0.09 s |
| driven (true pose) | 7.2 m |
| closest robot centre to a true wall | 0.119 m (Burger half width 0.089 m) |
| largest SLAM pose error against Gazebo truth | 0.14 m at 54.0 s |

Frontier clusters of 5 or more cells left at the end:

```
size  29 mid ( 1.49, -1.10)  no reachable free cell (free but not connected)
size  12 mid ( 1.71,  1.68)  no reachable free cell (all in padding or not free)
size  12 mid ( 1.64,  0.63)  no reachable free cell (all in padding or not free)
size  10 mid (-1.29,  1.49)  no reachable free cell (all in padding or not free)
size   6 mid ( 1.09,  0.32)  no reachable free cell (all in padding or not free)
```

The bag is in `~/bags/2026-10-10-lab3_maze_narrow-trail-2` and is not in git.
