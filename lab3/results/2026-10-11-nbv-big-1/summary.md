# 2026-10-11-nbv-big-1

World `lab3_maze`. Code: NBV planner, before the stall fix. All numbers [gazebo], one run.

Failed: 14.5 of about 52 m2, 74 stalls, 303 back-offs, time limit. Stalled goals were picked again and again.

| Item | Value |
|---|---|
| `exploration finished` printed | no |
| goals (plan lines) | 121 |
| arrived / stalled / not reachable / path is old | 27 / 74 / 0 / 19 |
| back-offs in the follower | 303 |
| known area at the end | 14.5 m2 |
| `known` first above 90 % of the final value | 94.0 s after the first plan |
| longest / median `plan_time` | 0.28 / 0.23 s |
| driven (true pose) | 75.0 m |
| closest robot centre to a true wall | 0.148 m (Burger half width 0.089 m) |
| largest SLAM pose error against Gazebo truth | 0.12 m at 1804.0 s |

Frontier clusters of 5 or more cells left at the end:

```
none
```

The bag is in `~/bags/2026-10-11-nbv-big-1` and is not in git.
