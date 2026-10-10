# 2026-10-10-lab3_maze-2

World `lab3_maze`. Code: commit cf4fab1. All numbers [gazebo], one run.

SLAM held (max error 0.17 m). The only entrance to the right half was crossed off after stalls, so the right half was never explored. The stalls were the same creep on the slow-down ramp as small run 7.

| Item | Value |
|---|---|
| `exploration finished` printed | yes, 689 s after the first plan |
| goals (plan lines) | 48 |
| arrived / stalled / not reachable / path is old | 27 / 5 / 0 / 16 |
| back-offs in the follower | 11 |
| known area at the end | 28.8 m2 |
| `known` first above 90 % of the final value | 598.0 s after the first plan |
| longest / median `plan_time` | 0.53 / 0.11 s |
| driven (true pose) | 62.7 m |
| closest robot centre to a true wall | 0.101 m (Burger half width 0.089 m) |
| largest SLAM pose error against Gazebo truth | 0.17 m at 449.0 s |

Frontier clusters of 5 or more cells left at the end:

```
size  26 mid ( 3.30, -2.39)  retired near (3.3, -2.39)
```

The bag is in `~/bags/2026-10-10-lab3_maze-2` and is not in git.
