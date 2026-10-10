"""Near Identity Diffeomorphism (NID) position controller.

The TurtleBot3 is non-holonomic: it cannot move sideways, so its centre cannot be
driven directly by a linear controller. NID controls a point P a fixed distance L
ahead of the robot instead. Turning the robot sweeps P sideways, so P behaves like a
holonomic point and a simple proportional controller works on it.

    x_p = x + L*cos(theta)
    y_p = y + L*sin(theta)

Inverting the transform gives the robot's two inputs from a desired velocity of P:

    v     =  u_x*cos(theta) + u_y*sin(theta)
    omega = (-u_x*sin(theta) + u_y*cos(theta)) / L

Outer loop is proportional control on P's position error:

    u = k_p * (goal - P)
"""

import math

from .controller_base import Command, Controller, Pose2D
from .geometry import clamp

# Setpoint is where P should end up. Closed loop on P is exactly linear; the robot's
# centre settles L behind the goal.
GOAL_DEFINES_OFFSET_POINT = 'offset_point'

# Setpoint is where the robot's centre should end up. The goal is shifted forward by
# L before the error is computed, so the centre converges on it, at the cost of exact
# linearity in the closed loop.
GOAL_DEFINES_ROBOT_CENTRE = 'robot_centre'

GOAL_DEFINES_CHOICES = (GOAL_DEFINES_OFFSET_POINT, GOAL_DEFINES_ROBOT_CENTRE)


class NIDController(Controller):
    """Proportional position control of the NID offset point P."""

    def __init__(
        self,
        offset_L: float,
        k_p: float,
        max_linear_velocity: float,
        max_angular_velocity: float,
        goal_tolerance: float,
        goal_defines: str = GOAL_DEFINES_OFFSET_POINT,
    ) -> None:
        if offset_L <= 0.0:
            # L = 0 divides by zero in compute() and collapses P back onto the
            # non-holonomic centre.
            raise ValueError('nid_offset_L must be greater than zero, got %r' % offset_L)
        if goal_defines not in GOAL_DEFINES_CHOICES:
            raise ValueError(
                'goal_defines must be one of %r, got %r' % (GOAL_DEFINES_CHOICES, goal_defines)
            )
        self.offset_L = offset_L
        self.k_p = k_p
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        self.goal_tolerance = goal_tolerance
        self.goal_defines = goal_defines

    def offset_point(self, pose: Pose2D):
        """Position of the controlled point P, given the robot's pose."""
        return (
            pose.x + self.offset_L * math.cos(pose.theta),
            pose.y + self.offset_L * math.sin(pose.theta),
        )

    def tracking_error(self, pose: Pose2D, goal_x: float, goal_y: float):
        """Error vector from P (or the shifted goal, under robot_centre) to the goal."""
        p_x, p_y = self.offset_point(pose)
        if self.goal_defines == GOAL_DEFINES_ROBOT_CENTRE:
            goal_x = goal_x + self.offset_L * math.cos(pose.theta)
            goal_y = goal_y + self.offset_L * math.sin(pose.theta)
        e_x = goal_x - p_x
        e_y = goal_y - p_y
        return e_x, e_y, math.hypot(e_x, e_y)

    def compute(self, pose: Pose2D, goal_x: float, goal_y: float) -> Command:
        """Velocity command driving P toward the goal."""
        e_x, e_y, distance = self.tracking_error(pose, goal_x, goal_y)

        if distance <= self.goal_tolerance:
            return Command(0.0, 0.0)

        # Outer loop: desired velocity of P, in the world frame.
        u_x = self.k_p * e_x
        u_y = self.k_p * e_y

        # Scale the demand vector, not v/omega separately afterwards, so a saturated
        # command still points at the goal.
        speed = math.hypot(u_x, u_y)
        if speed > self.max_linear_velocity and speed > 0.0:
            scale = self.max_linear_velocity / speed
            u_x *= scale
            u_y *= scale

        # NID transform.
        cos_t = math.cos(pose.theta)
        sin_t = math.sin(pose.theta)
        v = u_x * cos_t + u_y * sin_t
        omega = (-u_x * sin_t + u_y * cos_t) / self.offset_L

        v = clamp(v, -self.max_linear_velocity, self.max_linear_velocity)
        omega = clamp(omega, -self.max_angular_velocity, self.max_angular_velocity)
        return Command(v, omega)
