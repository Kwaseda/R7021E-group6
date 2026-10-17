# Lab 1: report notes

Source notes for the Lab 1 report of group 6. Written 2026-10-17 from the three bags recorded
on the TurtleBot3 Burger robot turtle2 on 2026-09-30, a Gazebo re-run of task 4 and a desk
simulation of the wall follower. Figures are in `plots/`, made by `plots/make_figures.py`.
Bags and videos are in the shared Drive folder, under `bags/` and `videos/`.

## Contributions

1. Dominic Addo: the ROS 2 package (position controller, figure of eight, wall follower,
   launch and parameter files), the experiments in simulation and on the robot, and the
   analysis of the bags.
2. Abdalrahman Nasser: the plotting and video scripts, the figures, and the formatting of
   the report.

## Block diagram

[PLACEHOLDER: block diagram of the control loop and the ROS implementation. It has to show
the levels of control, the type of feedback and the type of control input.]

## The robot model, and how the control input is computed

The Burger robot is a differential drive robot. With wheel speeds v_R and v_L and a wheel
separation of 0.160 m, the robot's forward speed and turn rate are

    v = (v_R + v_L) / 2,    omega = (v_R - v_L) / 0.160

and its pose (x, y, theta) in the odometry frame moves as the unicycle model:

    x_dot = v cos(theta),   y_dot = v sin(theta),   theta_dot = omega

We control at the kinematic level. The OpenCR board runs the wheel speed loop, so our node
only sends v and omega, as a `geometry_msgs/TwistStamped` on `/cmd_vel` at 20 Hz. The
feedback is the robot's own odometry on `/odom`, from the wheel encoders.

The robot cannot move sideways, so its centre cannot be steered straight at a goal. For that
reason, we use the near identity diffeomorphism (NID) and control a point P that is
L = 0.10 m ahead of the wheel axle:

    x_P = x + L cos(theta),   y_P = y + L sin(theta)

P can move in any direction, so a proportional law works on it, u = k_p (goal - P) with
k_p = 0.8. The speed of u is capped at 0.22 m/s, and inverting the NID transform gives the
two inputs we send:

    v     =  u_x cos(theta) + u_y sin(theta)
    omega = (-u_x sin(theta) + u_y cos(theta)) / L

In words: v is the part of u along the heading, and omega is the sideways part divided by
L. The node stops when P is within 0.05 m of the goal.

## Task 1: position control

Figure: `plots/lab1_task1.png`. Video: `videos/lab1_task1_robot_2x.webm`.

We sent five goals: (1, 0), (0, 0), (0, 1), (1, 1) and (1, 0). P reached every one of them
within the 0.05 m tolerance. The robot's centre stopped about 0.10 to 0.14 m short each
time, because the goal is where P stops (`goal_defines: offset_point`), and P is 0.10 m
ahead of the centre. For the first goal, "1 m forward", the centre stopped at x = 0.862 m
and P at x = 0.961 m. Running with `goal_defines: robot_centre` shifts the goal forward by
L, so that the centre lands on it.

The first goal is missing from the bag. It was published once, before the recorder had
picked up the topic, so we know it only from where P stopped.

### Why the robot curves before driving back to a point

The teacher asked why the robot curves a little and corrects its heading when it is sent
back to a coordinate. When the goal is behind the robot, the error vector u points
backwards. Its part along the heading is negative, so v is negative and the robot reverses,
and its sideways part divided by L = 0.10 m gives a large omega. In the bag, v went down to
about -0.22 m/s and omega reached about 2.2 rad/s, which is 0.22 / 0.10. The robot
therefore reverses while it swings round, which is the curve, and it drives forward once P
points at the goal. It never turns on the spot first, because NID does not control the
heading at all.

## Task 2: the figure of eight

Figures: `plots/lab1_task2.png`, `plots/lab1_task2_delay.png`, `plots/lab1_task2_error.png`.
Video: `videos/lab1_task2_robot.webm`.

### How we get the eight

A trajectory node publishes a moving goal on `/new_position` at 20 Hz. The goal follows a
Gerono lemniscate, x = a sin(s) and y = b sin(2s) with s = 2 pi t / 50 s, so y does two
oscillations while x does one and the curve crosses itself once in the middle. On the robot
we used a = 0.5 m and b = 0.25 m (an eight of 1.0 by 0.5 m) to fit the floor space, for two
laps. The position controller simply chases the goal.

### What we saw

The robot drew the right shape, but late. A proportional controller only acts on an error
that already exists, so it follows a moving goal with a delay of about 1 / k_p = 1.25 s. In
Gazebo the best-fit delay was 1.25 s. On the robot it was 1.35 s, and with that delay
removed the remaining error of P is only about 0.014 m. The extra 0.10 s fits the 130 to
270 ms that we measured between a command and the robot's response. The mean error of P
was 0.083 m, against v / k_p = 0.061 / 0.8 = 0.076 m predicted.

There were two more things in the bag. First, the robot reversed about 5 cm at the start,
because P started 0.10 m ahead of the first goal. Second, odometry stopped reaching the
laptop for more than 0.5 s five times while the robot was moving, the longest for 2.6 s.
Each time, the controller stopped the robot (its odometry timeout is 0.5 s), and the
tracking error jumped once it restarted.

## Task 4: wall following

Figures: `plots/lab1_task4_sim.png`, `plots/lab1_task4_sim_block.png`,
`plots/lab1_task4_robot_start.png`. Videos: `videos/lab1_task4_sim_2x.webm`,
`videos/lab1_task4_robot_start_pose_4x.webm`.

### How the lidar lets the robot follow a wall

The lidar spins and measures the distance to whatever is around the robot, one beam per
degree. The node looks at the beams on the right side and takes the closest one as the
distance to the wall. If the robot is too far from the wall, it turns toward it; if it is
too close, it turns away (omega = k_p (distance - 0.5 m), with k_p = 1.2). Two beams 40
degrees apart also give the angle of the wall, so the robot can straighten up before it
drifts. If something is closer than 0.35 m in front, it stops and turns on the spot. It
stops for good when odometry says it is back where the lap started.

### What happened in the lab

The teacher placed a 3D printed maze block of about 0.25 by 0.25 m and asked us to follow
it as the wall. The robot followed the block for a while, but from too far away. Then it
made loops and never got all the way round and back. It never hit anything, also not the
teacher who stepped in front of it, because of the 0.35 m front stop. Our task 4 bag holds
only 129 s of the robot standing at its start position, so the motion itself was not
recorded.

### Why it did that

Our follower was tuned for room walls, not for a small block. There are three reasons:

1. The setpoint of 0.5 m is a normal distance from a room wall but a large one from a 25
   cm block, so the robot looked far away.
2. Each time the block left the right-side beams at one of its corners, the code treated
   it as the largest error it handles (0.5 m) and turned at 1.2 x 0.5 = 0.6 rad/s at
   0.15 m/s. That is a circle of 0.25 m radius, while going round the block at 0.5 m
   needs about 0.63 m. The robot turned too tightly and made loops.
3. The lap only starts when the two-beam fit sees a straight wall, and a 0.25 m block is
   too small for both beams to hit it. The lap never started, so the robot could never
   decide that it was back.

We checked this in a desk simulation with our own wall follower code, a simulated lidar,
the 0.25 m block and the settings from the lab day. With the block on the right, as in the
lab, the robot went round it about 1.9 times in 90 s at a median 0.46 m from it, and never
stopped. Over 96 different start poses it went round in 50 percent of the runs and closed
the loop in none of them.

### The wall follower in simulation

The teacher asked us to show the wall following in simulation. In Gazebo, with the same
code and settings as on the lab day, the robot found the wall of the room, followed it
round all four sides and stopped when it was back within 0.30 m of its lap start, after
about 15.4 m. So the wall follower works on walls that are long enough for it, and the
problem in the lab was the small block.

## Snapshots

Frames from the RViz replays, top-down in the odometry frame. Each shows the robot model,
the path so far (blue), the current goal (orange) and the laser scan (green):

1. `plots/lab1_task1_snapshot.png`: task 1, back at the last goal after the square of goals.
2. `plots/lab1_task2_snapshot.png`: task 2, near the end of the second lap of the eight.
3. `plots/lab1_task4_sim_snapshot.png`: task 4 in Gazebo, closing the lap round the room.
4. `plots/lab1_task4_robot_snapshot.png`: task 4 on the robot, the start pose beside the wall.

The videos were made by replaying each bag into RViz with `tools/rviz_replay/rviz_replay.py`.
The task 1 and task 4 simulation videos play at 2x speed, and the task 4 robot video at 4x,
since the robot stands still in it.

## Reflection questions

1. Differences between simulation and the real robot: the real robot answers a command
   130 to 270 ms later and coasts 1 to 2 cm after a stop, odometry arrived over Wi-Fi with
   gaps of up to 3.7 s (none in simulation), and at the full 0.22 m/s the robot made only
   about 16 percent of the small turns that it was told to make, against 94 percent at the
   lower speeds of the eight. The real lidar also returns 0 where Gazebo returns infinity
   for a beam that hits nothing, and our code ignores both.
2. How comparable the results are: close where the data is clean. The eight lagged 1.35 s
   on the robot against 1.25 s in Gazebo, and every goal in task 1 was reached in both. The
   differences are in timing and in turning at full speed.
3. Where the robot gets its odometry: from the robot itself. The TurtleBot's driver,
   started by `robot.launch.py`, turns the wheel encoder counts into `/odom`. It drifts and
   it starts at zero wherever the robot is when the driver starts.
4. What sim to real needed: `use_sim_time` set to false, the right `ROS_DOMAIN_ID` (32 for
   turtle2), `TwistStamped` on `/cmd_vel`, and lidar ranges of 0 treated as no echo. The
   run showed two more: a speed below 0.22 m/s, so that the wheels can still turn, and an
   odometry timeout that tolerates the lab Wi-Fi.

## ROS, in short

The teacher asked what ROS is and how the laptop and the robot talk. ROS 2 is a framework
in which programs (nodes) exchange messages on named topics. A node publishes on a topic
and every node that subscribes to it receives the message, with no direct link between
them. On the robot, `robot.launch.py` starts the driver, which publishes `/odom`, `/scan`
and `/tf` and subscribes to `/cmd_vel`. On the laptop, our node subscribes to `/odom` and
`/new_position` and publishes `/cmd_vel`. They find each other automatically over the
network, as long as both use the same `ROS_DOMAIN_ID` (32 for turtle2). Turning the robot on
is not enough, though: we first SSH into it and start `robot.launch.py`, and only then can
commands from the laptop move it.
