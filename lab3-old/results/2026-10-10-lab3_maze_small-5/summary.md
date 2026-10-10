# 2026-10-10-lab3_maze_small-5

World `lab3_maze_small`. Code: travel back to 0.2 / 0.3, + kp_yaw 1.0 and max_w 0.6. All numbers [gazebo], one run.

No stalls and one back-off, but SLAM jumped from 0.15 m to 1.72 m error in 4 s: a false loop closure.

| Item | Value |
|---|---|
| `exploration finished` printed | yes, 144 s after the first plan |
| goals (plan lines) | 13 |
| arrived / stalled / not reachable / path is old | 11 / 0 / 0 / 2 |
| back-offs in the follower | 1 |
| known area at the end | 12.3 m2 |
| `known` first above 90 % of the final value | 114.0 s after the first plan |
| longest / median `plan_time` | 0.08 / 0.04 s |
| driven (true pose) | 13.5 m |
| closest robot centre to a true wall | 0.128 m (Burger half width 0.089 m) |
| largest SLAM pose error against Gazebo truth | 1.72 m at 129.0 s |

Frontier clusters of 5 or more cells left at the end:

```
size  26 mid (-0.57,  2.32)  no reachable free cell (free but not connected)
size   7 mid ( 1.14, -0.51)  no reachable free cell (all in padding or not free)
```

The bag is in `~/bags/2026-10-10-lab3_maze_small-5` and is not in git.
