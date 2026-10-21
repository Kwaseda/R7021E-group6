"""The wall following law and the loop closure test, as plain Python.

No ROS imports, so both classes can be tested at a desk. wall_follower_node.py reads
the scan and odometry and hands the numbers in.
"""

import math
from typing import Optional, Tuple

from .controller_base import Command
from .geometry import clamp, wrap_to_pi

#: Bearing of the sector the wall is expected in, radians, per side. Straight ahead is
#: zero and positive is to the left, matching the scan's own angle_min/angle_increment.
SIDE_BEARING = {'right': -0.5 * math.pi, 'left': 0.5 * math.pi}

#: Sign that turns a distance error into a turn rate for each side. Following on the
#: right, being too far from the wall turns the robot right (negative omega); on the
#: left, the same error turns it left.
SIDE_SIGN = {'right': -1.0, 'left': 1.0}


def wall_geometry(a: float, b: float, delta: float) -> Tuple[float, float]:
    """Perpendicular distance to a straight wall, and the robot's angle into it.

    A single range reading can't distinguish approaching from receding, so distance
    alone isn't enough to steer on. Two beams fix a line, which has both a distance
    and an angle.

    Take two beams on the wall side: `a` perpendicular to the robot, `b` at `delta`
    radians toward the front. A beam at bearing theta meets a straight wall at

        r(theta) = d / sin(psi - theta)

    where d is the perpendicular distance and psi is the wall's direction relative to
    the robot's heading. Writing that for both beams and eliminating d gives

        psi = atan2(a - b*cos(delta), b*sin(delta))
        d   = a*cos(psi)

    psi is zero when the robot runs parallel to the wall, positive when heading into a
    wall on its right.

    Exact for a straight wall; wrong at a corner, where the two beams hit different
    walls, which is what the front sector and cornering state handle instead.
    """
    psi = math.atan2(a - b * math.cos(delta), b * math.sin(delta))
    return a * math.cos(psi), psi


class WallFollowLaw:
    """Proportional control on distance to the wall on one side.

    One measured number does the steering: distance to the wall on the chosen side, at
    a constant forward speed. No map, no plan, no global frame.

    Three states:

    Acquiring, when the nearest wall is further away than the proportional band
    reaches: point at the nearest wall and drive at it. Without this, a distant wall
    and a near wall produce the same capped turn rate, so the robot holds a constant
    speed and turn rate indefinitely -- a circle that never reaches a wall.

    Following, when a wall is within reach and nothing is close in front:

        omega = side_sign * (k_p * (distance - setpoint) - k_heading * psi)

    Two proportional terms: distance error, and heading error from wall_geometry().
    Distance alone can't tell approaching from receding.

    Cornering, when something is inside front_stop_distance: stop and pivot away at
    corner_turn_rate. An inside corner is a wall the side sector can't see yet, not a
    distance error on it, so driving forward while turning would clip it.
    """

    def __init__(
        self,
        follow_side: str,
        setpoint: float,
        k_p: float,
        k_heading: float,
        speed: float,
        front_stop_distance: float,
        corner_turn_rate: float,
        max_distance_error: float,
        acquire_turn_gain: float,
        max_linear_velocity: float,
        max_angular_velocity: float,
    ) -> None:
        if follow_side not in SIDE_SIGN:
            raise ValueError(
                'follow_side must be "right" or "left", got %r' % (follow_side,)
            )
        self.follow_side = follow_side
        self.side_sign = SIDE_SIGN[follow_side]
        self.side_bearing = SIDE_BEARING[follow_side]
        self.setpoint = setpoint
        self.k_p = k_p
        self.k_heading = k_heading
        self.speed = min(speed, max_linear_velocity)
        self.front_stop_distance = front_stop_distance
        self.corner_turn_rate = corner_turn_rate
        self.max_distance_error = max_distance_error
        self.acquire_turn_gain = acquire_turn_gain
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity

    @property
    def acquire_distance(self) -> float:
        """Distance beyond which there is nothing to follow, metres.

        Beyond setpoint + max_distance_error the error is capped anyway, so the law
        has no information left and the acquiring state takes over instead.
        """
        return self.setpoint + self.max_distance_error

    def compute(
        self,
        side_distance: Optional[float],
        front_distance: Optional[float],
        closest: Optional[Tuple[float, float]] = None,
        geometry: Optional[Tuple[float, float]] = None,
    ) -> Tuple[Command, str]:
        """Velocity command and the name of the state that produced it.

        `side_distance` / `front_distance` are None when the sector held no valid
        beam -- a real reading, not a failure (e.g. an outside corner briefly loses
        the wall from the side sector).

        `closest` is the nearest valid beam in the whole scan; it distinguishes "wall
        briefly out of the side sector" from "no wall nearby," which need opposite
        responses.

        `geometry` is the (distance, angle) pair from wall_geometry(), or None if
        either beam it needs was missing.
        """
        if front_distance is not None and front_distance < self.front_stop_distance:
            # Turn away from the wall in place.
            omega = -self.side_sign * self.corner_turn_rate
            return self._clamped(0.0, omega), 'corner'

        if closest is not None and closest[0] > self.acquire_distance:
            # No wall within reach of the proportional band: steer toward the
            # nearest wall and drive at it.
            bearing_error = wrap_to_pi(closest[1])
            omega = clamp(
                self.acquire_turn_gain * bearing_error,
                -self.corner_turn_rate,
                self.corner_turn_rate,
            )
            return self._clamped(self.speed, omega), 'acquiring'

        if geometry is not None:
            # Two beams: distance and angle. The caller always picks the forward beam
            # on the side being followed, so psi is already in one convention on both
            # sides: positive means heading into the wall.
            distance, psi = geometry
        elif side_distance is not None:
            # One beam only -- distance, undamped. Half a measurement steers better
            # than none for the tick or two it lasts.
            distance, psi = side_distance, 0.0
        else:
            # Wall has left the side sector but something is still close: an outside
            # corner. Treat it as the largest error the law acts on.
            distance, psi = self.setpoint + self.max_distance_error, 0.0

        # Cap the error, not the resulting omega, so the response stays proportional
        # inside the band and constant outside it.
        error = clamp(distance - self.setpoint,
                      -self.max_distance_error, self.max_distance_error)
        omega = self.side_sign * (self.k_p * error - self.k_heading * psi)
        return self._clamped(self.speed, omega), 'following'

    def _clamped(self, v: float, omega: float) -> Command:
        """Apply the robot's actuator bounds. Nothing leaves this class without it."""
        return Command(
            clamp(v, -self.max_linear_velocity, self.max_linear_velocity),
            clamp(omega, -self.max_angular_velocity, self.max_angular_velocity),
        )


def max_fittable_wall_angle(beam_separation: float) -> float:
    """Largest wall angle the two beam fit can resolve, radians, for a given separation.

    Eliminating d from the two beam equations gives the forward range in terms of the
    perpendicular one:

        b = a / (cos(delta) + sin(delta) * tan(psi))

    which is singular at psi = -(pi/2 - delta): there the forward beam runs parallel to
    the wall and never reaches it. Approaching that angle, b runs away to infinity, so a
    forward beam that misses the surface entirely and lands on something far behind is
    indistinguishable from a wall raking away at just under the singular angle. The fit
    reports a confident, badly wrong psi, and the follower steers on it.

    Everything at or beyond this angle is unusable, so the accepted range has to stay
    strictly inside it. The caller keeps a margin as well: at delta = 40 degrees the
    singularity is at 50 degrees, and even 45 degrees still admits b = 8*a.
    """
    return 0.5 * math.pi - beam_separation


class LapStart:
    """Decides when the follower has settled onto the wall, so a lap can be measured.

    The loop test compares a pose against the pose the lap started from, so that start
    pose has to be one the robot will actually pass again: a point on the track the
    follower settles into, at the heading it settles into.

    Being at the right distance is not enough. A robot switched on pointing into the
    room is momentarily at the setpoint distance while swinging round to face along the
    wall, and a lap referenced to that pose is referenced to a heading the robot never
    holds again -- it returns to the same place tens of degrees off and the heading gate
    rejects a lap it really did complete.

    So settling requires the wall fit as well: distance at the setpoint *and* running
    parallel to the wall, held for `settle_ticks` consecutive ticks. The fit is what
    supplies the heading, which is the part distance alone cannot see.

    Latches: once settled, it stays settled, so a corner later in the lap cannot move
    the reference the lap is being measured against.
    """

    def __init__(
        self,
        setpoint: float,
        distance_tolerance: float,
        angle_tolerance: float,
        settle_ticks: int,
    ) -> None:
        self.setpoint = setpoint
        self.distance_tolerance = distance_tolerance
        self.angle_tolerance = angle_tolerance
        self.settle_ticks = max(1, int(settle_ticks))
        self.held = 0
        self.ready = False

    def update(
        self,
        state: str,
        side_distance: Optional[float],
        geometry: Optional[Tuple[float, float]],
    ) -> bool:
        """Feed one tick. True once the follower counts as settled on the wall."""
        if self.ready:
            return True
        settled_now = (
            state == 'following'
            and side_distance is not None
            and abs(side_distance - self.setpoint) <= self.distance_tolerance
            and geometry is not None
            and abs(geometry[1]) <= self.angle_tolerance
        )
        self.held = self.held + 1 if settled_now else 0
        self.ready = self.held >= self.settle_ticks
        return self.ready


class LoopCloseDetector:
    """Decides when the robot has returned to its start, travelling the same way.

    Detects that the odometry estimate has returned to the start pose, not a true
    loop closure -- odometry drifts, and this has no map to correct against.

    Three conditions:

    - Travelled at least `min_distance`. Without it, the test fires on tick one, since
      the robot starts within any tolerance of its own position.
    - Within `tolerance` metres of the start position.
    - Within `heading_tolerance` radians of the start heading. Without this, driving
      out along a wall and back down the same wall satisfies "near the start" without
      having gone around anything.

    Cannot catch a figure of eight, which returns to its start pose having enclosed
    nothing -- that needs a map, which a wall follower doesn't build.
    """

    def __init__(self, tolerance: float, min_distance: float, heading_tolerance: float) -> None:
        self.tolerance = tolerance
        self.min_distance = min_distance
        self.heading_tolerance = heading_tolerance
        self.start = None
        self.previous = None
        self.travelled = 0.0
        self.closed = False

    def update(self, x: float, y: float, theta: float) -> bool:
        """Feed one odometry pose. True once the loop has closed."""
        if self.start is None:
            self.start = (x, y, theta)
            self.previous = (x, y)
            return False

        # Path length accumulated between samples, not straight-line distance from
        # start (which stays small on a circular route).
        self.travelled += math.hypot(x - self.previous[0], y - self.previous[1])
        self.previous = (x, y)

        if self.closed:
            return True
        if self.travelled < self.min_distance:
            return False

        near = math.hypot(x - self.start[0], y - self.start[1]) <= self.tolerance
        aligned = abs(wrap_to_pi(theta - self.start[2])) <= self.heading_tolerance
        self.closed = near and aligned
        return self.closed

    def error_to_start(self, x: float, y: float) -> float:
        """Distance from the start position, metres. The task 4 acceptance number."""
        if self.start is None:
            return float('nan')
        return math.hypot(x - self.start[0], y - self.start[1])
