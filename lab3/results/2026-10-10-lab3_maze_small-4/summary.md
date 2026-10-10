# 2026-10-10-lab3_maze_small-4

World `lab3_maze_small`. Code: + SLAM minimum_travel 0.1 m / 0.15 rad (reverted after this run). All numbers [gazebo], one run.

SLAM broke: pose error jumped to 1.81 m and the map smeared outside the maze. The change was reverted.

| Item | Value |
|---|---|
| `exploration finished` printed | yes, 221 s after the first plan |
| goals (plan lines) | 14 |
| arrived / stalled / not reachable / path is old | 7 / 2 / 0 / 5 |
| back-offs in the follower | 7 |
| known area at the end | 11.8 m2 |
| `known` first above 90 % of the final value | 181.0 s after the first plan |
| longest / median `plan_time` | 0.05 / 0.03 s |
| driven (true pose) | 18.3 m |
| closest robot centre to a true wall | 0.107 m (Burger half width 0.089 m) |
| largest SLAM pose error against Gazebo truth | 1.81 m at 212.0 s |

Frontier clusters of 5 or more cells left at the end:

```
size  12 mid ( 1.30, -1.06)  no reachable free cell (free but not connected)
size   9 mid (-3.01, -0.87)  no reachable free cell (free but not connected)
size   8 mid (-3.25, -1.06)  no reachable free cell (all in padding or not free)
size   8 mid ( 0.75, -1.09)  no reachable free cell (free but not connected)
size   6 mid (-2.20,  0.36)  no reachable free cell (all in padding or not free)
```

The bag is in `~/bags/2026-10-10-lab3_maze_small-4` and is not in git.
