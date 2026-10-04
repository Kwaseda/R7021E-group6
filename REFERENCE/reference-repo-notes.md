# The reference repository — how to use it without stealing from it

`https://github.com/willmerw/R7021E-` — a classmate's work, posted publicly and
generously. 14 commits, labs 1 to 4.

Dominic's framing, and it is the right one: **learn from it, do not copy it.**

---

## The protocol — follow this or the repo becomes a liability

1. **Attempt first, read second.** For any task, you write your own version before opening
   their file for that task. Reading first and then "writing your own" produces their
   solution from memory with the variable names changed. That is copying with extra steps,
   and you will not be able to defend it any better than last time.
2. **Read it to compare, not to acquire.** The question when you open their file is never
   "what did they write?" but "what did they do *differently from me*, and which of us is
   right?" Sometimes it will be them. Sometimes it will be you — there are at least six
   places below where your own old code was better.
3. **Never copy a line verbatim.** Not even a short one. If their approach is better, close
   the file, then write your version from your understanding of the idea. If you can't write
   it without the file open, you don't understand it yet, which is the actual problem.
4. **Architecture is fair game, implementation is not.** "Four single-purpose nodes meeting
   at one topic" is a design idea you can learn and adopt. The specific maths inside
   `wall_follow.py` is their work.
5. **If anything of theirs influenced a design decision, say so in `NOTES.md`.** One line:
   *"goal-source-as-topic structure informed by a classmate's public repo."* Then if you are
   ever asked, you have already answered. Hiding the influence is what turns learning into
   misconduct.
6. **Do not open labs 3 and 4 at all yet.** Those you have not attempted. Reading them now
   costs you the only chance you get to learn them properly.

Claude's obligation here: refuse to transcribe their code into Dominic's files, and when
their approach is the better one, describe the *idea* and make him implement it.

---

## What their Lab 1 actually is

One package, `turtlebot_control`. Four nodes. No `config/` directory. No YAML. Two small
launch files. **300 lines total.**

| File | Lines | Job |
|---|---|---|
| `cmd_vel_sender.py` | 80 | the controller: `/odom` + `/new_position` → `/cmd_vel` |
| `position_sender.py` | 58 | walks a goal around a figure-8, publishes `/new_position` |
| `closest_wallpt.py` | 81 | `/scan` → nearest wall point in world frame → `/wall_pt` |
| `wall_follow.py` | 81 | `/wall_pt` → a goal 0.4 m off the wall, ahead → `/new_position` |

Compare: your Lab 1 was 2 packages, 5 YAML files, 14 modules, 2505 lines, for the same
three tasks.

### The idea worth having

Everything is "publish a goal point". The controller never knows or cares where the goal
came from. So:

- **Task A** — you publish a goal by hand from a terminal.
- **Task B** — `position_sender` publishes goals stepping around a figure-8.
- **Task C** — `closest_wallpt` finds the nearest wall point; `wall_follow` pushes that point
  out by `wall_dist` along the robot→wall direction, then adds a tangential offset (the
  point rotated 90°) to place the goal **ahead along the wall**. Wall following becomes
  *chase a carrot* — no new control law at all.

One controller, three goal sources, and each node explainable in one sentence. That is why
it reads well and why the instructor was satisfied. **Steal this idea. Write your own code
for it.**

### Their figure-8 is cruder than yours and that is interesting

They precompute a list of points — two circles of radius `d`, centres at `(x+d, y)` and
`(x−d, y)` — and step through it by index on a 0.7 s timer. No time parameterisation, no
lemniscate, no velocity profile; when it reaches the end it wraps to zero. Your Gerono
lemniscate (`x = W·sin s`, `y = H·sin 2s`) is mathematically better and you had actual
reasoning behind `lap_period`.

So this is not "they were better at everything". They were better at *packaging*. The
instructor graded packaging and defensibility, and on those you lost.

---

## What NOT to copy — their real defects

Several of these your own old code got right. Keeping your engineering substance while
adopting their file economy is the whole target.

1. **No velocity saturation.** `cmd_vel_sender.py` sets `z_speed = ang_diff` directly. A
   normalised angle error reaches ±π ≈ 3.14 rad/s, which **exceeds the Burger's 2.84 rad/s
   limit**. There is no clamp anywhere. Your saturation work is genuinely better — keep it.
2. **No stop on shutdown.** `main()` is `init / spin / destroy / shutdown` with no
   `try/finally`. On Ctrl-C the last `/cmd_vel` stands and the robot keeps going. Your run
   sheet already refuses to run unimproved nodes on hardware for exactly this reason.
3. **Unfiltered laser scan.** `closest_wallpt.py` does `np.argmin(ranges)` on the raw array.
   `ranges` contains `0.0` for beams that hit nothing, so the "closest wall" can be a
   phantom at zero distance. The study guide says to filter `inf`, `0`, and out-of-range
   first. This is a real bug, not a style point.
4. **`np.linspace(angle_min, angle_max, len(ranges))`** for beam angles — off by one
   increment across a full 360° scan. Use `angle_min + i * angle_increment`.
5. **No loop-closure detection in `wall_follow.py`.** The task asks for one full loop and
   then stop. Theirs follows a wall indefinitely. Your lap-detection work, including the
   measured 0.3 m tolerance, answers a requirement theirs doesn't.
6. **Constant forward speed** (`x_speed = 0.05`) regardless of how large the heading error
   is, so the robot arcs rather than turning then driving. Also 0.05 m/s is slow enough to
   make a figure-8 take a long time.
7. **Magic numbers inline** — `0.4`, `0.05`, `0.3`, `0.7` sit in the middle of the maths with
   no `declare_parameter` and no comment. The instructor did not punish it, but it means you
   cannot tune in a lab session without editing code.
8. **Dead imports in all four files** — `from std_msgs.msg import String`, never used.
   Commented-out `declare_parameter` lines left in.
9. **`print()` instead of `self.get_logger()`** in `wall_follow.py`.
10. **`build/` and `install/` committed to git** — 1159 files including 7 MB compiled
    binaries. Your `.gitignore` is correct and theirs is absent.

### One thing you plainly did better

They have no equivalent of your saturation finding — the measurement that unclamped NID
commands 0.92 m/s and 19.95 rad/s and flips the robot. That is a real experimental result
with a number behind it, it answers sim-to-real question 4 directly, and it is worth more in
a report than a tidier figure-8. Bring it with you.

---

## Open question you must decide and be able to justify

**`/new_position`: `Pose` or `PoseStamped`?**

- The course material and the lab instructions say **`geometry_msgs/Pose`** — that is what
  `ros2 topic pub --once /new_position geometry_msgs/Pose "{position: {x: 0.5, y: 0.0}}"`
  sends, and it is the command the instructions give you.
- Your classmate used **`PoseStamped`** throughout. The advantage is real: `PoseStamped` has
  a header, so you can set `frame_id = 'odom'` and RViz will draw the goal directly — which
  is why they needed no `goal_marker_node`, and you built one.

They are not interchangeable: publishing one type to a subscriber expecting the other means
nothing arrives, **with no error message**. Pick one, use it everywhere, and know what you
would type to test it by hand.

Default to `Pose`, because it is what the instructions specify and what the demo command
sends — then solve the RViz problem separately if you need the goal visible. But if you
choose `PoseStamped`, that is defensible too; just be ready to say why, and be sure your
hand-typed test command matches.

---

## Lab 2 — what to note now, read properly later

Their Lab 2 is one package, `turtle_mpc`, two files: `mpc_point.py` (210 lines) and
`mpc_traj.py` (247). Again no YAML. There is also a `do_mpc.ipynb` — they prototyped the MPC
in a notebook before putting it in a node, which is a good habit and much faster than
debugging an optimiser through ROS.

Two conventions visible without reading the maths:

- Module-level functions `defineTBotModel()` and `defineTBotMPC(model, ts, N, x_goal, y_goal)`
  build the do-mpc objects; the node calls them in `__init__`. Keeps the optimiser setup out
  of the node body without needing a separate file.
- They publish **two** `nav_msgs/Path` topics: `/trajectory` (where the robot has actually
  been) and `/mpc_trajectory` (the predicted horizon from `mpc.data.prediction`). Drawing the
  predicted horizon in RViz is the single best way to show an MPC is working, and it makes a
  far better report figure than a plot of the path alone. **Do this.**

Lab 2 is the one that must be redone for marks. Do not open their MPC files until you have
your own node attempting to solve.
