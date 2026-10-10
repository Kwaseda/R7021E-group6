# 2026-10-10-lab3_maze-1

World `lab3_maze`. Code: as written at the desk (commit 210b3af). All numbers [gazebo], one run.

Same failure as small run 1: three stalls, goals crossed off, the run ended at 7.8 m2 of about 52 m2.

| Item | Value |
|---|---|
| `exploration finished` printed | yes, 119 s after the first plan |
| goals (plan lines) | 11 |
| arrived / stalled / not reachable / path is old | 7 / 3 / 0 / 1 |
| back-offs in the follower | 0 |
| known area at the end | 7.8 m2 |
| `known` first above 90 % of the final value | 91.0 s after the first plan |
| longest / median `plan_time` | 0.09 / 0.05 s |
| driven (true pose) | 9.7 m |
| closest robot centre to a true wall | 0.11 m (Burger half width 0.089 m) |
| largest SLAM pose error against Gazebo truth | 0.39 m at 119.0 s |

Frontier clusters of 5 or more cells left at the end:

```
size  21 mid (-0.71, -0.78)  retired near (-0.71, -0.78)
```

The bag is in `~/bags/2026-10-10-lab3_maze-1` and is not in git.
