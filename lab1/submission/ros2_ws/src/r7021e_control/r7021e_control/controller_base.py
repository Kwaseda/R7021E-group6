"""The seam between "what the robot is doing" and "what velocity to send".

Plain Python: no rclpy, no topics, no publishing. controller_node.py owns all of
that. The split lets the control law be tested at a desk and swapped for the Lab 2
MPC by changing which class the node constructs.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    """Where the robot is: the three states of the unicycle model.

        x, y     position in the odometry frame, metres
        theta    heading, radians

    Read off nav_msgs/Odometry, with theta recovered from the quaternion.
    """

    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class Command:
    """What is sent to the robot: the two inputs of the unicycle model.

        v        body forward speed, m/s
        omega    turn rate, rad/s

    These become twist.linear.x and twist.angular.z. No third field: that is the
    non-holonomic constraint, no input produces sideways motion.
    """

    v: float
    omega: float


class Controller:
    """A control law: current pose and desired position in, wheel commands out.

    Deliberately narrow: receives a state and a goal, returns a command. Does not
    subscribe, publish, log, or own a timer.
    """

    def compute(self, pose: Pose2D, goal_x: float, goal_y: float) -> Command:
        """Return the velocity command that drives the robot toward the goal."""
        raise NotImplementedError

    def reset(self) -> None:
        """Discard any internal state. Called when the goal changes discontinuously.

        The proportional NID law is memoryless, so this does nothing today. It's on
        the interface because an MPC's warm start, or an integral term, would be
        wrong to keep across a jump in the setpoint.
        """
