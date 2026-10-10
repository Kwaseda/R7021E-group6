# 2026-10-10-lab3_maze-3

World `lab3_maze`. Code: commit aa65d6f (ramp fix); the stop fix f632b52 came after and does not change driving. All numbers [gazebo], one run.

About 85 % of the maze. The top-right corridor mouth stalled more than the two retry rounds allow, so the block behind it was not explored.

| Item | Value |
|---|---|
| `exploration finished` printed | yes, 1223 s after the first plan |
| goals (plan lines) | 85 |
| arrived / stalled / not reachable / path is old | 45 / 10 / 0 / 30 |
| back-offs in the follower | 27 |
| known area at the end | 44.0 m2 |
| `known` first above 90 % of the final value | 1018.0 s after the first plan |
| longest / median `plan_time` | 3.77 / 0.6 s |
| driven (true pose) | 113.6 m |
| closest robot centre to a true wall | 0.121 m (Burger half width 0.089 m) |
| largest SLAM pose error against Gazebo truth | 0.28 m at 367.0 s |

Frontier clusters of 5 or more cells left at the end:

```
size  14 mid ( 3.41,  2.57)  retired near (3.41, 2.57)
```

The bag is in `~/bags/2026-10-10-lab3_maze-3` and is not in git.
