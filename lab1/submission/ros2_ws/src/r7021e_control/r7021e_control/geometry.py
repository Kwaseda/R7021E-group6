"""Small geometric helpers shared by the Lab 1 nodes.

Plain Python, no ROS imports, so these can be unit tested without a running node.
"""

import math


def clamp(value: float, lower: float, upper: float) -> float:
    """Return value limited to [lower, upper]."""
    return max(lower, min(upper, value))


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle in radians to (-pi, pi].

    Needed because 179 - (-179) degrees is 2 degrees, not 358.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract the yaw of a quaternion, in radians.

    A ground robot only rotates about the vertical axis, so only yaw is recovered;
    roll and pitch are discarded. Standard z-y-x Euler extraction reduced to its yaw
    term, matching tf_transformations.euler_from_quaternion(...)[2].
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float):
    """Build a unit quaternion (x, y, z, w) for a rotation of yaw about z."""
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))
