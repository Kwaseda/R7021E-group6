# Handoff: write the Lab 2, 3 and 4 reports

Written 2026-10-17. Lab 1 is finished and is the worked example to copy. Everything you need
is already in this repo; do not re-derive results, do not re-run the robot.

## Read these first, in this order

1. `lab1/report_notes.md`, how Dominic writes. Match this voice exactly.
2. `lab1/make_report.py`, the builder script. Copy it per lab, change the content.
3. `lab1/Lab_1_report_group6.docx`, the finished output.
4. `lab1/plots/make_block_diagram.py`, the diagram style. Copy it per lab.

## How Dominic writes, and it is not optional

Plain declarative sentences. Numbers with units, always. No marketing adjectives, no "crucial",
no "it is important to note". He explains mechanisms with a literal "In words:" line after an
equation. He states failures directly and then explains the cause rather than softening it.
British spelling. Sentence case headings.

Every .md you produce gets audited against `anthropic-skills:humanizer` before you hand it
over: no em dashes, no AI vocabulary, sparse bold, sentence-case headings.

He has asked repeatedly for short and direct. Answer first, evidence second.

## Toolchain, already solved

- No node, no pandoc. `python-docx` is installed (`pip install --user --break-system-packages python-docx`).
- Render to check: `soffice --headless --convert-to pdf X.docx` then `pdftoppm -jpeg -r 70 X.pdf page`, then read the jpgs. Always look at the pages.
- Figures: `doc.add_picture(path, width=Inches(5.1))`. Bigger than that and a single figure eats a page.
- Diagrams: matplotlib in pixel coordinates, saved as both .png and .svg. Palette and conventions in `lab1/plots/make_block_diagram.py`: white ground, `#1a1a1a` outlines at 1.2 px, plain rectangles for what the course provides, pastel fills (`#dae8fc`/`#6c8ebf` blue, `#f8cecc`/`#b85450` red, `#e6d98f`/`#b89b2e` sand) for blocks the group wrote, dashed rectangles for groupings, white chips on arrow labels. Keep the style identical across all four labs.

## Lab 2

Source: `lab2/report_notes.md` (215 lines), figures in `lab2/plots/` (11 PNGs already made),
bags in `lab2/bags/`. Tools: `lab2/plots/plot_rosbag.py` and `lab2/plots/sim_task.py`.

The brief wants:

1. The MPC setup per task: parameters, cost, constraints. The numbers are in the `TASKS` table
   in `lab2/ros2_ws/src/r7021e_lab2/r7021e_lab2/mpc_controller.py` and in the notes.
2. 2D plots per task showing robot position, goal points, bounding constraints and obstacles.
   `plot_rosbag.py` already draws all four of those.
3. **An extra simulation task that does not exist yet.** One obstacle, fixed goal, three cases:
   vary sampling time, and vary prediction horizon. All three plotted together, with an
   explanation of what changes and why. Use `sim_task.py --task 2 --t_step .. --n_horizon ..`
   and overlay the three paths on one axis. The explanation is already in the notes: horizon
   reach is `n_horizon * t_step * v_max`, which is 0.44 m at the defaults, and `t_step` also
   sets how coarsely the obstacle constraint is sampled.
4. Video: RViz with bounds, obstacles, goal, current position, path, and the predicted states.
   `/mpc_prediction` is the predicted horizon and is already published. Plus a second video of
   the three extra cases.

Known truths from the notes, state them plainly: task 4 never ran on the robot because the
robot started outside its boundary box and a hard state bound made step zero infeasible; task 2
failed because a 1.105 m keep-out needs about 2.2 m of detour against 0.44 m of reach; task 3
ran but with the goal at roughly (1.8, 0.5) so only one of the two constraints did any work.
The fixes are soft constraints, delay compensation and holding the last command, all measured.

## Lab 3

Source: `lab3/report_notes.md` (185 lines), `lab3/LOG.md`, results in `lab3/results/`
(four run folders), `lab3/STUDY/`, video in `lab3/video/`. There is no `lab3/plots/` yet, so
figures have to be made from the run folders.

Dominic's instruction for this one: **start from the original plan, then say what changed.**
The plan is `~/Downloads/r7021e-lab3-plan.docx` (and `lab3/PLAN.md` in this repo, which is the
same plan). So the structure is: what we set out to build, what we actually built and why it
differs, a new block diagram of the implemented stack, then the explanations.

The brief wants a single PDF with the lab group and members named. The instructions PDF is
`lab3/Lab_Instructions_Exploration.pdf`; its report section asks for four things specifically:

1. The ROS 2 implementation and node structure, inputs, outputs, workflow.
2. How the planner was made, with pseudocode.
3. Which exploration method, with the information gain equations.
4. Problems encountered during real-time implementation.

Around three pages with figures, focused on task 4, briefly covering tasks 1 to 3. A video link
is required.

**The block diagram is a deliverable here**, not decoration, because question 1 asks for the
node structure. Build it the same way as Lab 1's, showing the provided nodes
(`frontier_detector_node`, `path_follower_node`, slam_toolbox) as plain boxes and the filled-in
`navigation_node` as the pastel one.

## Lab 4

Source: `lab4/report_notes.md` (185 lines), `lab4/LOG.md`, `lab4/RUNSHEET.md`, `lab4/desk/`,
`lab4/STUDY/`, video in `lab4/video/`. No `lab4/plots/` yet.

Instructions: `~/Documents/R7021E_Lab_4.pdf` (also at `~/r7021e_2026/docs/labs/R7021E_Lab_4.pdf`).
Read it before writing; it has not been read yet in any session.

Submission is the report as a PDF, **plus the code, plus any video recordings**. So the zip
contains the `lab4/ros2_ws/src/` tree as well as the PDF.

## Things that are missing and that only Dominic can supply

- Video share links for every lab. The files live in his Google Drive folder, not in this repo,
  because Canvas will not take bags of that size. Lab 1's docx has `[VIDEO LINK]` placeholders
  for the same reason.
- Abdalrahman Nasser's student ID. Lab 1 has `[STUDENT ID]` as a placeholder.
- Group number and full member list for Lab 3, which the brief asks for explicitly.

Ask for these once, early, and leave clearly marked placeholders rather than blocking.

## Standing rules for this repo

- Dominic runs every `ros2`, `colcon`, `gz` and `rviz2` command himself. Read his output, do
  not execute them.
- `CLAUDE.md`, `PROPOSAL.md` and `lab2/PLAN.md` are gitignored on purpose. Do not commit them.
- Commit and push after each lab's report is built. The remote is
  `github.com/Kwaseda/R7021E-group6`, private, branch `main`.
- Do not add tasks, packages or structure that the briefs do not ask for. The first version of
  this course's work was rejected for exactly that.
