"""Shared, dependency-free motion logic.

This module is pure Python (standard library only). It is imported both by the
ROS 2 nodes in this package and by the standalone Tkinter simulator in
``sim/``, so the robot behaves identically in both places.

Contents
--------
* :class:`Pose`              - 2D pose (x, y, theta).
* :func:`integrate`          - differential-drive / unicycle kinematics.
* :func:`twist_to_wheels`    - (v, w) -> (left, right) wheel speeds.
* :func:`wheels_to_twist`    - (left, right) wheel speeds -> (v, w).
* :class:`TeleopController`  - keyboard -> (v, w) velocity command.
* :class:`ShapeFollower`     - open-loop square / circle path generator.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

TWO_PI = 2.0 * math.pi


def normalize_angle(angle: float) -> float:
    """Wrap an angle to the range [-pi, pi)."""
    return (angle + math.pi) % TWO_PI - math.pi


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


# --------------------------------------------------------------------------- #
# Kinematics
# --------------------------------------------------------------------------- #
@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


def integrate(pose: Pose, v: float, w: float, dt: float) -> Pose:
    """Advance ``pose`` by applying linear speed ``v`` (m/s) and angular speed
    ``w`` (rad/s) for ``dt`` seconds.

    Uses the exact solution for constant (v, w) - the robot moves along a
    circular arc - so results do not drift with large time steps.
    """
    if abs(w) < 1e-9:
        x = pose.x + v * dt * math.cos(pose.theta)
        y = pose.y + v * dt * math.sin(pose.theta)
    else:
        r = v / w
        new_theta = pose.theta + w * dt
        x = pose.x + r * (math.sin(new_theta) - math.sin(pose.theta))
        y = pose.y - r * (math.cos(new_theta) - math.cos(pose.theta))
    return Pose(x, y, normalize_angle(pose.theta + w * dt))


def twist_to_wheels(v: float, w: float, wheel_base: float) -> Tuple[float, float]:
    """Convert body velocity to (left, right) wheel linear speeds in m/s."""
    half = wheel_base / 2.0
    return v - w * half, v + w * half


def wheels_to_twist(v_left: float, v_right: float, wheel_base: float) -> Tuple[float, float]:
    """Convert (left, right) wheel linear speeds in m/s to body (v, w)."""
    return (v_right + v_left) / 2.0, (v_right - v_left) / wheel_base


# --------------------------------------------------------------------------- #
# Keyboard teleoperation
# --------------------------------------------------------------------------- #
TELEOP_HELP = """\
Keyboard teleop
---------------
  w / Up     : increase forward speed
  s / Down   : decrease forward speed (reverse)
  a / Left   : increase left turn rate
  d / Right  : increase right turn rate
  space / x  : stop
"""


class TeleopController:
    """Turns key presses into a (v, w) command.

    Each press nudges the speed by a fixed step, so the robot keeps moving
    after the key is released (like ``turtlebot3_teleop``). This works the
    same in a terminal (ROS node) and in a GUI (simulator).
    """

    KEY_ALIASES = {
        'up': 'w', 'down': 's', 'left': 'a', 'right': 'd', ' ': 'x', 'space': 'x',
    }

    def __init__(self, linear_step: float = 0.1, angular_step: float = 0.2,
                 max_linear: float = 1.0, max_angular: float = 2.0):
        self.linear_step = linear_step
        self.angular_step = angular_step
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.v = 0.0
        self.w = 0.0

    def handle_key(self, key: str) -> Optional[Tuple[float, float]]:
        """Process one key. Returns the new (v, w), or None if the key is unused."""
        key = self.KEY_ALIASES.get(key.lower(), key.lower())
        if key == 'w':
            self.v += self.linear_step
        elif key == 's':
            self.v -= self.linear_step
        elif key == 'a':
            self.w += self.angular_step
        elif key == 'd':
            self.w -= self.angular_step
        elif key == 'x':
            self.v = 0.0
            self.w = 0.0
        else:
            return None
        self.v = round(clamp(self.v, -self.max_linear, self.max_linear), 6)
        self.w = round(clamp(self.w, -self.max_angular, self.max_angular), 6)
        return self.v, self.w

    def stop(self) -> None:
        self.v = 0.0
        self.w = 0.0


# --------------------------------------------------------------------------- #
# Autonomous shapes
# --------------------------------------------------------------------------- #
@dataclass
class Segment:
    v: float         # linear speed (m/s)
    w: float         # angular speed (rad/s)
    duration: float  # seconds


def square_segments(size: float, linear_speed: float, angular_speed: float,
                    loops: int = 1) -> List[Segment]:
    """Drive forward ``size`` metres, turn left 90 degrees, repeat 4 x loops."""
    forward = Segment(linear_speed, 0.0, size / linear_speed)
    turn = Segment(0.0, angular_speed, (math.pi / 2.0) / angular_speed)
    return [forward, turn] * (4 * loops)


def circle_segments(radius: float, linear_speed: float, loops: int = 1) -> List[Segment]:
    """Drive a full counter-clockwise circle of ``radius`` metres, ``loops`` times."""
    w = linear_speed / radius
    return [Segment(linear_speed, w, loops * TWO_PI * radius / linear_speed)]


SHAPES = ('square', 'circle')


class ShapeFollower:
    """Time-based (open-loop) generator of velocity commands for a shape.

    Call :meth:`command` with the elapsed time since the start; it returns
    ``(v, w, done)``. No feedback is needed, so it works with any robot that
    accepts ``cmd_vel`` - including turtlesim and the bundled simulator.
    """

    def __init__(self, shape: str = 'square', size: float = 2.0,
                 linear_speed: float = 0.5, angular_speed: float = 0.8,
                 loops: int = 1):
        if shape not in SHAPES:
            raise ValueError(f"unknown shape '{shape}', expected one of {SHAPES}")
        if size <= 0 or linear_speed <= 0 or angular_speed <= 0 or loops < 1:
            raise ValueError('size, speeds must be > 0 and loops >= 1')
        self.shape = shape
        if shape == 'square':
            self.segments = square_segments(size, linear_speed, angular_speed, loops)
        else:
            self.segments = circle_segments(size, linear_speed, loops)
        self.total_duration = sum(s.duration for s in self.segments)

    def command(self, elapsed: float) -> Tuple[float, float, bool]:
        if elapsed < 0:
            return 0.0, 0.0, False
        t = elapsed
        for seg in self.segments:
            if t < seg.duration:
                return seg.v, seg.w, False
            t -= seg.duration
        return 0.0, 0.0, True

    def time_to_next_change(self, elapsed: float) -> float:
        """Seconds until the command changes (next segment boundary or the end)."""
        t = max(elapsed, 0.0)
        for seg in self.segments:
            if t < seg.duration:
                return seg.duration - t
            t -= seg.duration
        return float('inf')

    def simulate(self, dt: float = 0.01, start: Optional[Pose] = None) -> List[Pose]:
        """Run the whole shape through :func:`integrate` and return the poses."""
        pose = start or Pose()
        poses = [pose]
        # Integrate segment by segment so segment boundaries are hit exactly.
        for seg in self.segments:
            remaining = seg.duration
            while remaining > 1e-12:
                step = min(dt, remaining)
                pose = integrate(pose, seg.v, seg.w, step)
                poses.append(pose)
                remaining -= step
        return poses
