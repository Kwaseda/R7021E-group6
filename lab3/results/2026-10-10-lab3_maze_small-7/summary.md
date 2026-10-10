# 2026-10-10-lab3_maze_small-7

World `lab3_maze_small`. Code: same as run 6 (commit cf4fab1). All numbers [gazebo], one run.

Same code as the full run 6, and it failed like run 1: three stalls at one corridor mouth. The follower crept toward stop_distance on the slow-down ramp (v 0.00 to 0.01) and never counted as blocked, so it never backed off.

| Item | Value |
|---|---|
| `exploration finished` printed | yes, 165 s after the first plan |
| goals (plan lines) | 12 |
| arrived / stalled / not reachable / path is old | 6 / 3 / 0 / 3 |
| back-offs in the follower | 9 |
| known area at the end | 5.6 m2 |
| `known` first above 90 % of the final value | 35.0 s after the first plan |
| longest / median `plan_time` | 0.06 / 0.05 s |
| driven (true pose) | 6.0 m |
| closest robot centre to a true wall | 0.153 m (Burger half width 0.089 m) |
| largest SLAM pose error against Gazebo truth | 0.06 m at 38.0 s |

Frontier clusters of 5 or more cells left at the end:

```
size  16 mid (-1.56, -0.42)  retired near (-1.56, -0.42)
```

The bag is in `~/bags/2026-10-10-lab3_maze_small-7` and is not in git.
