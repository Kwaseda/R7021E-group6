#!/usr/bin/env python3
"""Build the Lab 1 report .docx from report_notes.md content and plots/."""

import os
from docx import Document
from docx.shared import Pt, Inches, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH

HERE = os.path.dirname(os.path.abspath(__file__))
PLOTS = os.path.join(HERE, 'plots')
OUT = os.path.join(HERE, 'Lab_1_report_group6.docx')

doc = Document()
st = doc.styles['Normal']
st.font.name = 'Calibri'
st.font.size = Pt(11)
for s in ('Heading 1', 'Heading 2', 'Heading 3'):
    doc.styles[s].font.color.rgb = RGBColor(0x1a, 0x1a, 0x1a)

fig_n = [0]


def h(text, level=1):
    doc.add_heading(text, level=level)


def p(text, italic=False, center=False):
    par = doc.add_paragraph()
    run = par.add_run(text)
    run.italic = italic
    if center:
        par.alignment = WD_ALIGN_PARAGRAPH.CENTER
    return par


def eq(text):
    par = doc.add_paragraph()
    par.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = par.add_run(text)
    run.font.name = 'Consolas'
    run.font.size = Pt(10.5)
    return par


def fig(name, caption, width=5.1):
    path = os.path.join(PLOTS, name)
    if not os.path.exists(path):
        p('[MISSING FIGURE: %s]' % name, italic=True)
        return
    doc.add_picture(path, width=Inches(width))
    doc.paragraphs[-1].alignment = WD_ALIGN_PARAGRAPH.CENTER
    fig_n[0] += 1
    cap = p('Figure %d: %s' % (fig_n[0], caption), italic=True, center=True)
    cap.runs[0].font.size = Pt(9.5)


def bullets(items, numbered=False):
    for it in items:
        doc.add_paragraph(it, style='List Number' if numbered else 'List Bullet')


# ------------------------------------------------------------------ page 1
t = doc.add_heading('R7021E Advanced Robotics and AI', level=0)
t.alignment = WD_ALIGN_PARAGRAPH.CENTER
s = p('Lab 1: position control, a figure of eight, and wall following', center=True)
s.runs[0].bold = True
s.runs[0].font.size = Pt(14)
p('Group 6', center=True)
p('')

h('Contributions', 1)
p('Dominic Jesse Kwame Addo (domadd-2): the ROS 2 package, that is the position '
  'controller, the figure of eight trajectory generator, the wall follower, and the launch '
  'and parameter files. The experiments in simulation and on the robot. The analysis of the '
  'recorded bags.')
p('Abdalrahman Nasser ([STUDENT ID]): the plotting and video scripts, the figures, and the '
  'formatting of the report.')

p('')
h('Videos', 1)
p('The videos are larger than 10 MB, so they are shared online. Each one is a replay of a '
  'recorded bag into RViz, showing the robot model, the path so far, the current goal and '
  'the laser scan at every instant.')
bullets([
    'Task 1, position control on the robot (2x speed): [VIDEO LINK]',
    'Task 2, the figure of eight on the robot: [VIDEO LINK]',
    'Task 4, wall following in Gazebo (2x speed): [VIDEO LINK]',
    'Task 4, the robot at its start pose in the lab (4x speed): [VIDEO LINK]',
])

doc.add_page_break()

# ------------------------------------------------------------------ theory
h('1. ROS 2', 1)
p('ROS 2 is a framework in which programs, called nodes, exchange messages on named topics. '
  'A node publishes on a topic and every node that subscribes to that topic receives the '
  'message. There is no direct link between them, so nodes can be started, stopped and '
  'replaced on their own.')
p('On the robot, robot.launch.py starts the driver. The driver publishes /odom, /scan and '
  '/tf, and subscribes to /cmd_vel. On the laptop, our node subscribes to /odom and '
  '/new_position and publishes /cmd_vel. The two machines find each other automatically over '
  'the network as long as both use the same ROS_DOMAIN_ID, which was 32 for turtle2. Turning '
  'the robot on is not enough on its own: we first open an SSH shell on the robot and start '
  'robot.launch.py, and only then can commands from the laptop move it.')

h('2. Block diagram', 1)
p('Figure 1 shows the two control loops we ran. Both are closed loops at the kinematic level. '
  'The control input we send is a forward speed v and a turn rate omega, published as a '
  'geometry_msgs/TwistStamped on /cmd_vel at 20 Hz.')
fig('lab1-block-diagram.png', 'The two control loops. The position controller is driven by a '
    'goal on /new_position and closes the loop on odometry. The wall follower has no goal and '
    'closes the loop on the laser scan.')
p('There are three levels of control in the system. Level 1 is ours and is the only one we '
  'wrote: it turns a goal into v and omega. Level 2 is the wheel speed controller in the '
  'OpenCR firmware on the robot, which turns v and omega into the speed of each wheel. Level '
  '3 is the motors themselves. We never see levels 2 and 3, which is the point of sending a '
  'velocity rather than a motor command.')
p('The feedback differs between the two loops. The position controller is fed back the pose '
  'the robot computes from its wheel encoders and IMU, on /odom. The wall follower is fed '
  'back the raw distances from the lidar, on /scan, and never uses the pose to steer. It '
  'uses odometry only to notice that a lap has closed.')

h('3. The robot model, and how the control input is computed', 1)
p('The Burger is a differential drive robot. With wheel speeds v_R and v_L and a wheel '
  'separation of 0.160 m, its forward speed and turn rate are')
eq('v = (v_R + v_L) / 2        omega = (v_R - v_L) / 0.160')
p('and its pose (x, y, theta) in the odometry frame moves as the unicycle model:')
eq('x_dot = v cos(theta)     y_dot = v sin(theta)     theta_dot = omega')
p('We control at the kinematic level, so our node only sends v and omega and the OpenCR board '
  'runs the wheel speed loop.')
p('The robot cannot move sideways, so its centre cannot be steered straight at a goal. For '
  'that reason we use the near identity diffeomorphism, and control a point P that sits '
  'L = 0.10 m ahead of the wheel axle:')
eq('x_P = x + L cos(theta)     y_P = y + L sin(theta)')
p('P can move in any direction, so a proportional law works on it, with k_p = 0.8:')
eq('u = k_p (goal - P)')
p('The magnitude of u is capped at 0.22 m/s. Inverting the transform gives the two inputs we '
  'publish:')
eq('v     =   u_x cos(theta) + u_y sin(theta)')
eq('omega = (-u_x sin(theta) + u_y cos(theta)) / L')
p('In words: v is the part of u that points along the current heading, and omega is the part '
  'that points sideways, divided by L. The node stops when P is within 0.05 m of the goal.')

doc.add_page_break()

# ------------------------------------------------------------------ task 1
h('4. Task 1: position control', 1)
p('We sent five goals in turn: (1, 0), (0, 0), (0, 1), (1, 1) and (1, 0). P reached every one '
  'of them within the 0.05 m tolerance.')
fig('lab1_task1.png', 'Task 1 on the robot. The path of the robot centre and of the '
    'controlled point P, against the goals that were sent.')
p('The robot centre stopped about 0.10 to 0.14 m short of each goal. That is expected: the '
  'goal is where P stops, and P is 0.10 m ahead of the centre. For the first goal, one metre '
  'forward, the centre stopped at x = 0.862 m and P at x = 0.961 m. Setting the parameter '
  'goal_defines to robot_centre shifts the goal forward by L, so that the centre lands on it '
  'instead.')
p('The first goal is missing from the bag. It was published once, before the recorder had '
  'matched the topic, so we know it only from where P stopped.')

h('Why the robot curves before driving back to a point', 2)
p('We were asked why the robot curves and corrects its heading when it is sent back to a '
  'coordinate behind it. When the goal is behind the robot, the error vector u points '
  'backwards. Its component along the heading is negative, so v is negative and the robot '
  'reverses. Its sideways component is divided by L = 0.10 m, which gives a large omega. In '
  'the bag, v reached about -0.22 m/s and omega about 2.2 rad/s, which is 0.22 / 0.10.')
p('The robot therefore reverses while it swings round, and that is the curve. It drives '
  'forward again once P points at the goal. It never turns on the spot first, because the '
  'near identity diffeomorphism does not control the heading at all.')

# ------------------------------------------------------------------ task 2
h('5. Task 2: the figure of eight', 1)
h('How the eight is generated', 2)
p('A trajectory node publishes a moving goal on /new_position at 20 Hz. The goal follows a '
  'Gerono lemniscate,')
eq('x = a sin(s)     y = b sin(2s)     s = 2 pi t / 50 s')
p('so y completes two oscillations while x completes one, and the curve crosses itself once '
  'in the middle. On the robot we used a = 0.5 m and b = 0.25 m, an eight of 1.0 by 0.5 m, to '
  'fit the floor space we had, for two laps. The position controller simply chases that goal, '
  'so no new control law is needed for this task.')
fig('lab1_task2.png', 'Task 2 on the robot. The path of P against the commanded figure of '
    'eight, for two laps.')

h('What we saw', 2)
p('The robot drew the right shape, but late. A proportional controller only acts on an error '
  'that already exists, so it follows a moving goal with a delay of about 1 / k_p = 1.25 s. '
  'In Gazebo the best fit delay was 1.25 s. On the robot it was 1.35 s, and once that delay '
  'is removed the remaining error of P is only about 0.014 m. The extra 0.10 s matches the '
  '130 to 270 ms we measured between a command and the robot responding to it.')
fig('lab1_task2_delay.png', 'The tracking error against an assumed delay. The minimum is at '
    '1.35 s on the robot, against 1.25 s in Gazebo.', width=4.9)
p('The mean error of P was 0.083 m, against v / k_p = 0.061 / 0.8 = 0.076 m predicted by the '
  'same argument.')
fig('lab1_task2_error.png', 'Tracking error over the run. The jumps follow the gaps in the '
    'odometry stream.', width=4.9)
p('Two other things are visible in the bag. The robot first reversed about 5 cm, because P '
  'started 0.10 m ahead of the first goal. And odometry stopped reaching the laptop for more '
  'than 0.5 s on five occasions while the robot was moving, the longest gap being 2.6 s. Each '
  'time, the controller stopped the robot, because its odometry timeout is 0.5 s, and the '
  'tracking error jumped once the stream restarted.')

doc.add_page_break()

# ------------------------------------------------------------------ task 4
h('6. Task 4: wall following', 1)
h('How the lidar lets the robot follow a wall', 2)
p('The lidar spins and measures the distance to whatever is around the robot, one beam per '
  'degree. The node reads the beams on the right side and takes the closest one as the '
  'distance to the wall. If the robot is too far from the wall it turns towards it, and if it '
  'is too close it turns away:')
eq('omega = k_p (distance - 0.50 m),   k_p = 1.2')
p('Two beams 40 degrees apart also give the angle of the wall, so the robot can straighten up '
  'before it drifts. If anything is closer than 0.35 m in front, it stops driving forward and '
  'turns on the spot. It stops for good when odometry says it is back where the lap started.')

h('What happened in the lab', 2)
p('The teacher placed a 3D printed maze block of about 0.25 by 0.25 m and asked us to follow '
  'it as the wall. The robot followed the block for a while, but from too far away. It then '
  'made loops and never got all the way round and back. It never hit anything, including the '
  'teacher when he stepped in front of it, because of the 0.35 m front stop. Our task 4 bag '
  'holds only 129 s of the robot standing at its start position, so the motion itself was not '
  'recorded.')
fig('lab1_task4_robot_start.png', 'Task 4 on the robot. The start pose beside the block, which '
    'is all the bag contains.', width=4.7)

h('Why it did that', 2)
p('The follower was tuned for room walls, not for a small block. There are three reasons.')
bullets([
    'The setpoint of 0.50 m is a normal distance from a room wall but a large one from a '
    '25 cm block, so the robot sat far away from it.',
    'Each time the block left the right-side beams at one of its corners, the code treated it '
    'as the largest error it handles, 0.5 m, and turned at 1.2 x 0.5 = 0.6 rad/s while driving '
    'at 0.15 m/s. That is a circle of 0.25 m radius, while going round the block at 0.50 m '
    'needs about 0.63 m. The robot turned too tightly and made loops.',
    'The lap only starts once the two-beam fit sees a straight wall, and a 0.25 m block is too '
    'small for both beams to land on. The lap never started, so the robot could never decide '
    'that it was back.',
], numbered=True)
p('We checked this in a desk simulation with our own wall follower code, a simulated lidar, '
  'the 0.25 m block and the settings from the lab day. With the block on the right, as in the '
  'lab, the robot went round it about 1.9 times in 90 s at a median distance of 0.46 m, and '
  'never stopped. Over 96 different start poses it went round in 50 percent of the runs and '
  'closed the loop in none of them.')
fig('lab1_task4_sim_block.png', 'The desk simulation with the 0.25 m block, reproducing the '
    'loops we saw in the lab.', width=4.7)

h('The wall follower in simulation', 2)
p('We were asked to show the wall following working in simulation. In Gazebo, with the same '
  'code and the same settings as on the lab day, the robot found the wall of the room, '
  'followed it round all four sides and stopped when it was back within 0.30 m of its lap '
  'start, after about 15.4 m. The wall follower therefore works on walls that are long enough '
  'for it, and the problem in the lab was the size of the block.')
fig('lab1_task4_sim.png', 'Task 4 in Gazebo. One full lap of the room, stopping where the lap '
    'started.')

doc.add_page_break()

# ------------------------------------------------------------------ snapshots
h('7. Snapshots from the videos', 1)
p('The frames below are taken from the RViz replays, seen from above in the odometry frame. '
  'Each shows the robot model, the path so far, the current goal and the laser scan.')
fig('lab1_task1_snapshot.png', 'Task 1, back at the last goal after the square of goals.',
    width=4.7)
fig('lab1_task2_snapshot.png', 'Task 2, near the end of the second lap of the eight.',
    width=4.7)
fig('lab1_task4_sim_snapshot.png', 'Task 4 in Gazebo, closing the lap round the room.',
    width=4.7)
fig('lab1_task4_robot_snapshot.png', 'Task 4 on the robot, the start pose beside the wall.',
    width=4.7)

doc.add_page_break()

# ------------------------------------------------------------------ questions
h('8. Reflections on the experiment', 1)

h('Do you observe any difference between controlling the robot in simulation and controlling '
  'the physical robot in real time?', 2)
p('Yes, in four ways that we could measure. The real robot answers a command 130 to 270 ms '
  'later and coasts 1 to 2 cm after a stop. Odometry arrived over Wi-Fi with gaps of up to '
  '3.7 s, where in simulation there were none. At the full 0.22 m/s the robot made only about '
  '16 percent of the small turns it was told to make, against 94 percent at the lower speeds '
  'used in the figure of eight. And the real lidar returns 0 for a beam that hits nothing, '
  'where Gazebo returns infinity; our code ignores both.')

h('How comparable is the robot’s actual performance to its performance in simulation?', 2)
p('Close, where the data is clean. The figure of eight lagged 1.35 s on the robot against '
  '1.25 s in Gazebo, and every goal in task 1 was reached in both. The differences are in '
  'timing, and in turning at full speed. The wall follower is the exception, and there the '
  'difference was not the robot but the object: it completed a lap of a room in Gazebo and '
  'could not complete a lap of a 0.25 m block in the lab, for the reasons in section 6.')

h('Where does the physical TurtleBot obtain its odometry information from?', 2)
p('From the robot itself. The driver started by robot.launch.py turns the wheel encoder '
  'counts into a pose and publishes it on /odom, with the onboard IMU helping the heading. It '
  'is dead reckoning, so it drifts, and it starts at zero wherever the robot happens to be '
  'when the driver starts. Nothing corrects it against the outside world.')

h('What changes, adjustments, or additional considerations are required for sim-to-real '
  'transfer?', 2)
p('Four were needed before anything moved: use_sim_time set to false, the right '
  'ROS_DOMAIN_ID, which was 32 for turtle2, TwistStamped rather than Twist on /cmd_vel, and '
  'lidar ranges of 0 treated as no echo. The run itself showed two more: a commanded speed '
  'below 0.22 m/s, so that the wheels can still turn when a turn is also demanded, and an '
  'odometry timeout loose enough to tolerate the lab Wi-Fi. Our 0.5 s timeout stopped the '
  'robot five times in one run.')

doc.save(OUT)
print('wrote', OUT)
