# 2026-10-10-lab3_maze_narrow-infl0105

World `lab3_maze_narrow`. Code: commit f632b52, inflation_radius 0.105. All numbers [gazebo], one run.

The robot drove through the 0.40 m door into the middle room. Then the padding closed the door in the map, every other frontier was 'free but not connected', and the run finished with most of the world unexplored.

| Item | Value |
|---|---|
| `exploration finished` printed | yes, 47 s after the first plan |
| goals (plan lines) | 3 |
| arrived / stalled / not reachable / path is old | 3 / 0 / 0 / 0 |
| back-offs in the follower | 1 |
| known area at the end | 7.8 m2 |
| `known` first above 90 % of the final value | 17.0 s after the first plan |
| longest / median `plan_time` | 0.2 / 0.1 s |
| driven (true pose) | 4.0 m |
| closest robot centre to a true wall | 0.137 m (Burger half width 0.089 m) |
| largest SLAM pose error against Gazebo truth | 0.06 m at 52.0 s |

Frontier clusters of 5 or more cells left at the end:

```
size  51 mid (-1.42, -1.10)  no reachable free cell (free but not connected)
size  27 mid ( 1.46,  0.93)  no reachable free cell (free but not connected)
size  18 mid ( 1.20, -1.20)  no reachable free cell (free but not connected)
size  15 mid (-0.04,  1.31)  no reachable free cell (free but not connected)
size   7 mid ( 1.07, -1.34)  no reachable free cell (free but not connected)
size   6 mid ( 1.00,  1.42)  no reachable free cell (free but not connected)
```

The bag is in `~/bags/2026-10-10-lab3_maze_narrow-infl0105` and is not in git.
