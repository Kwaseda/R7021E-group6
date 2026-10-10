# 2026-10-10-lab3_maze_small-9

World `lab3_maze_small`. Code: commit aa65d6f (ramp fix); the stop fix f632b52 came after and does not change driving. All numbers [gazebo], one run.

Complete exploration.

| Item | Value |
|---|---|
| `exploration finished` printed | yes, 296 s after the first plan |
| goals (plan lines) | 21 |
| arrived / stalled / not reachable / path is old | 14 / 0 / 0 / 7 |
| back-offs in the follower | 5 |
| known area at the end | 15.6 m2 |
| `known` first above 90 % of the final value | 259.0 s after the first plan |
| longest / median `plan_time` | 0.98 / 0.08 s |
| driven (true pose) | 30.3 m |
| closest robot centre to a true wall | 0.105 m (Burger half width 0.089 m) |
| largest SLAM pose error against Gazebo truth | 0.27 m at 172.0 s |

Frontier clusters of 5 or more cells left at the end:

```
none
```

The bag is in `~/bags/2026-10-10-lab3_maze_small-9` and is not in git.
