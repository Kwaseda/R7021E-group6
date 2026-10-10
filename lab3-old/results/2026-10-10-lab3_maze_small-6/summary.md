# 2026-10-10-lab3_maze_small-6

World `lab3_maze_small`. Code: + do_loop_closing false (commit cf4fab1). All numbers [gazebo], one run.

First complete exploration. Map matches the true walls.

| Item | Value |
|---|---|
| `exploration finished` printed | yes, 254 s after the first plan |
| goals (plan lines) | 22 |
| arrived / stalled / not reachable / path is old | 17 / 0 / 0 / 5 |
| back-offs in the follower | 0 |
| known area at the end | 15.0 m2 |
| `known` first above 90 % of the final value | 223.0 s after the first plan |
| longest / median `plan_time` | 0.13 / 0.07 s |
| driven (true pose) | 26.8 m |
| closest robot centre to a true wall | 0.128 m (Burger half width 0.089 m) |
| largest SLAM pose error against Gazebo truth | 0.24 m at 145.0 s |

Frontier clusters of 5 or more cells left at the end:

```
none
```

The bag is in `~/bags/2026-10-10-lab3_maze_small-6` and is not in git.
