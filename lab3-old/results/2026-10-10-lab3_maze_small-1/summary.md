# 2026-10-10-lab3_maze_small-1

World `lab3_maze_small`. Code: as written at the desk (commit 210b3af). All numbers [gazebo], one run.

The follower froze at a wall corner (v and w zero) twice. Both goals were crossed off for good, they were the openings to the rest of the maze, and the run ended early. The bag was closed late: the recorder ignored SIGINT from a background script, so it covers dead time at the end.

| Item | Value |
|---|---|
| `exploration finished` printed | yes, 64 s after the first plan |
| goals (plan lines) | 7 |
| arrived / stalled / not reachable / path is old | 5 / 2 / 0 / 0 |
| back-offs in the follower | 0 |
| known area at the end | 5.5 m2 |
| `known` first above 90 % of the final value | 28.0 s after the first plan |
| longest / median `plan_time` | 0.09 / 0.08 s |
| driven (true pose) | 4.0 m |
| closest robot centre to a true wall | 0.0 m (Burger half width 0.089 m) |
| largest SLAM pose error against Gazebo truth | ? |

Frontier clusters of 5 or more cells left at the end:

```
size  14 mid (-1.56, -0.46)  retired near (-1.56, -0.46)
size   5 mid (-1.75,  0.29)  retired near (-1.7, 0.27)
```

The bag is in `~/bags/2026-10-10-lab3_maze_small-1` and is not in git.
