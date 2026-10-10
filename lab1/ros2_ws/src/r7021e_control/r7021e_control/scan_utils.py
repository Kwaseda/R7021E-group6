"""Reading a LaserScan without believing everything it says.

No ROS imports, so these can be unit tested against a hand-built scan.

Every function here gates on the same thing: an LDS-01 returns 0.0 or inf for a beam
that hit nothing, or hit something closer than its minimum range. A raw min() over
the array would return 0.0 for the whole run.
"""

import math
from typing import Optional, Sequence, Tuple

# One valid measurement: distance in metres, bearing in radians, index into ranges.
Measurement = Tuple[float, float, int]


def beam_angle(index: int, angle_min: float, angle_increment: float) -> float:
    """Bearing of beam `index`, radians, in the scan's own frame.

        theta_i = angle_min + i * angle_increment

    Positive is to the left, straight ahead is zero -- the ROS convention, matching
    the unicycle model's theta.
    """
    return angle_min + index * angle_increment


def is_valid(value: float, range_min: float, range_max: float) -> bool:
    """Report whether a range reading is a measurement rather than a failure."""
    return math.isfinite(value) and range_min <= value <= range_max


def closest_point(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
) -> Optional[Measurement]:
    """Closest valid beam in the scan, or None if the scan carries no valid beam.

        i* = argmin{ ranges[i] : range_min <= ranges[i] <= range_max }

    Returns the distance, the bearing of that beam, and its index. None is a real
    answer, not an error: a robot beyond the lidar's range sees no wall at all.
    """
    best: Optional[Measurement] = None
    for i, value in enumerate(ranges):
        if not is_valid(value, range_min, range_max):
            continue
        if best is None or value < best[0]:
            best = (float(value), beam_angle(i, angle_min, angle_increment), i)
    return best


def sector_min(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    centre: float,
    width: float,
) -> Optional[Measurement]:
    """Closest valid beam inside an angular sector, or None if the sector is empty.

    `centre` and `width` are radians: the sector spans centre +/- width/2. A sector
    rather than a single beam, since one beam can miss a gap or drop out entirely.
    """
    half = 0.5 * width
    best: Optional[Measurement] = None
    for i, value in enumerate(ranges):
        if not is_valid(value, range_min, range_max):
            continue
        bearing = beam_angle(i, angle_min, angle_increment)
        # Wrap the offset, not the bearing, so a sector behind the robot still works
        # when the scan runs 0 to 2*pi instead of -pi to pi.
        offset = math.atan2(math.sin(bearing - centre), math.cos(bearing - centre))
        if abs(offset) > half:
            continue
        if best is None or value < best[0]:
            best = (float(value), bearing, i)
    return best


def beam_at(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    bearing: float,
    window: float,
) -> Optional[Measurement]:
    """Return the valid beam pointing closest to `bearing`, within `window` of it.

    Different question from sector_min: this asks how far the wall is along one
    exact ray, which is what fixing a line from two beams needs. `window` allows for
    a dropped beam -- take the nearest valid neighbour rather than lose the reading.
    """
    best: Optional[Measurement] = None
    best_offset = None
    for i, value in enumerate(ranges):
        if not is_valid(value, range_min, range_max):
            continue
        theta = beam_angle(i, angle_min, angle_increment)
        offset = abs(math.atan2(math.sin(theta - bearing), math.cos(theta - bearing)))
        if offset > window:
            continue
        if best_offset is None or offset < best_offset:
            best_offset = offset
            best = (float(value), theta, i)
    return best
