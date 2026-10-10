# 2026-10-10-lab3_maze_narrow-trail-1

World `lab3_maze_narrow`. Code: f632b52 + trail-as-free fix (not committed). All numbers [gazebo], one run.

The robot entered the 0.40 m door room. The planner planned out, but the follower blocked 66 times at the door post: a 0.40 m door leaves 0.11 m each side.

| Item | Value |
|---|---|
| `exploration finished` printed | yes, 388 s after the first plan |
| goals (plan lines) | 21 |
| arrived / stalled / not reachable / path is old | 2 / 9 / 0 / 10 |
| back-offs in the follower | 66 |
| known area at the end | 8.1 m2 |
| `known` first above 90 % of the final value | 8.0 s after the first plan |
| longest / median `plan_time` | 0.17 / 0.09 s |
| driven (true pose) | 15.1 m |
| closest robot centre to a true wall | 0.164 m (Burger half width 0.089 m) |
| largest SLAM pose error against Gazebo truth | 0.08 m at 23.0 s |

Frontier clusters of 5 or more cells left at the end:

```
size  32 mid ( 1.21, -1.29)  no reachable free cell (free but not connected)
size  20 mid (-0.04,  1.33)  no reachable free cell (free but not connected)
size  16 mid (-1.46, -0.51)  no reachable free cell (free but not connected)
size  15 mid ( 1.16,  1.15)  no reachable free cell (free but not connected)
size   5 mid ( 1.00,  1.40)  no reachable free cell (all in padding or not free)
```

The bag is in `~/bags/2026-10-10-lab3_maze_narrow-trail-1` and is not in git.
