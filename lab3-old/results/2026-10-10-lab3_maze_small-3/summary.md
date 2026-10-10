# 2026-10-10-lab3_maze_small-3

World `lab3_maze_small`. Code: + look_ahead 0.12 m. All numbers [gazebo], one run.

Better map. The robot stuck in a dead-end cell: SLAM pose was 0.18 m off, and stayed off because SLAM only updates after 0.2 m or 0.3 rad. The path from the wrong pose ran into a wall post.

| Item | Value |
|---|---|
| `exploration finished` printed | yes, 252 s after the first plan |
| goals (plan lines) | 22 |
| arrived / stalled / not reachable / path is old | 14 / 6 / 0 / 2 |
| back-offs in the follower | 12 |
| known area at the end | 12.5 m2 |
| `known` first above 90 % of the final value | 67.0 s after the first plan |
| longest / median `plan_time` | 0.2 / 0.1 s |
| driven (true pose) | 13.4 m |
| closest robot centre to a true wall | 0.142 m (Burger half width 0.089 m) |
| largest SLAM pose error against Gazebo truth | 0.19 m at 237.0 s |

Frontier clusters of 5 or more cells left at the end:

```
size  15 mid (-0.28,  1.60)  retired near (-0.28, 1.6)
size  13 mid ( 1.10, -1.59)  retired near (1.1, -1.59)
```

The bag is in `~/bags/2026-10-10-lab3_maze_small-3` and is not in git.
