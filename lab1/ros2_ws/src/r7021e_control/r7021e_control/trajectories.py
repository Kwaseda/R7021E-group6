"""The figure of eight, as a function of time.

No ROS imports, so the path can be plotted and unit tested without a robot.
trajectory_node.py turns what this returns into a geometry_msgs/Pose.

Gerono lemniscate:

    x(s) = centre_x + width  * sin(s)
    y(s) = centre_y + height * sin(2s)          s = 2*pi * (elapsed / lap_period)

The double frequency in y makes the curve cross itself once, at the centre, giving a
figure of eight that spans exactly 2*width by 2*height.

Path properties at width=1.0, height=0.5 (config/trajectory.yaml defaults), computed
from the curve:

    arc length          6.097 m per lap
    tightest radius     0.209 m, at the crossing
    mean speed          0.122 m/s at a 50 s lap
    peak speed          0.175 m/s (80% of the Burger's 0.22 m/s limit)
    peak turn rate      0.40 rad/s (14% of the Burger's 2.84 rad/s limit)

The turn rate figure is a floor, not a ceiling: it's what the path itself demands of
something already on it. NID divides lateral tracking error by L, so a robot that
isn't exactly on the path demands more.
"""

import math
from typing import Tuple


def figure_eight(
    phase: float,
    width: float,
    height: float,
    centre_x: float = 0.0,
    centre_y: float = 0.0,
) -> Tuple[float, float]:
    """Point on the eight at `phase` radians. One lap is 2*pi of phase."""
    return (
        centre_x + width * math.sin(phase),
        centre_y + height * math.sin(2.0 * phase),
    )


class FigureEightTrajectory:
    """A figure of eight walked at a fixed rate, for a fixed number of laps.

    Holds the shape and the clock policy, nothing else.
    """

    def __init__(
        self,
        width: float,
        height: float,
        centre_x: float,
        centre_y: float,
        lap_period: float,
        laps: int,
        start_delay: float,
    ) -> None:
        if lap_period <= 0.0:
            raise ValueError('lap_period must be greater than zero, got %r' % lap_period)
        if laps <= 0:
            raise ValueError('laps must be at least one, got %r' % laps)
        self.width = width
        self.height = height
        self.centre_x = centre_x
        self.centre_y = centre_y
        self.lap_period = lap_period
        self.laps = laps
        self.start_delay = max(0.0, start_delay)

    @property
    def duration(self) -> float:
        """Seconds from start to the end of the last lap, including the delay."""
        return self.start_delay + self.laps * self.lap_period

    def point_at(self, elapsed: float) -> Tuple[float, float, bool]:
        """Setpoint at `elapsed` seconds after the node started.

        Returns x, y and whether the run has finished. Three phases:

        - Before start_delay: setpoint holds at the centre of the figure, giving the
          controller time to drive the robot onto the path before lap one starts.
        - During the laps: setpoint walks the curve at a constant rate of phase.
        - After the last lap: setpoint holds the final point rather than going silent.
        """
        if elapsed < self.start_delay:
            return self.point_at_phase(0.0) + (False,)

        running = elapsed - self.start_delay
        finished = running >= self.laps * self.lap_period
        if finished:
            running = self.laps * self.lap_period

        phase = 2.0 * math.pi * (running / self.lap_period)
        return self.point_at_phase(phase) + (finished,)

    def point_at_phase(self, phase: float) -> Tuple[float, float]:
        """Point on the eight at a given phase."""
        return figure_eight(phase, self.width, self.height, self.centre_x, self.centre_y)
