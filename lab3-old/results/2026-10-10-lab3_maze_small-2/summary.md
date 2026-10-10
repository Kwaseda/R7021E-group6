# 2026-10-10-lab3_maze_small-2

World `lab3_maze_small`. Code: + follower back-off, + stalled-goal retry. All numbers [gazebo], one run.

The back-off fired 23 times. The robot backed off and drove into the same corner again (look_ahead 0.20 m cut the corner). From about 109 s the robot did not move again.

| Item | Value |
|---|---|
| `exploration finished` printed | yes, 349 s after the first plan |
| goals (plan lines) | 23 |
| arrived / stalled / not reachable / path is old | 9 / 6 / 0 / 8 |
| back-offs in the follower | 23 |
| known area at the end | 11.5 m2 |
| `known` first above 90 % of the final value | 81.0 s after the first plan |
| longest / median `plan_time` | 0.12 / 0.07 s |
| driven (true pose) | 13.9 m |
| closest robot centre to a true wall | 0.11 m (Burger half width 0.089 m) |
| largest SLAM pose error against Gazebo truth | 0.23 m at 101.0 s |

Frontier clusters of 5 or more cells left at the end:

```
size  42 mid ( 2.26,  0.45)  no reachable free cell (free but not connected)
size  16 mid (-0.68,  1.53)  retired near (-0.68, 1.53)
size  13 mid ( 1.36,  0.78)  retired near (1.36, 0.78)
size   7 mid ( 2.24, -0.18)  no reachable free cell (all in padding or not free)
```

The bag is in `~/bags/2026-10-10-lab3_maze_small-2` and is not in git.
