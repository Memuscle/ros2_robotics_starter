"""Unit tests for the pure-Python motion module (no ROS needed).

    python3 -m pytest src/robotics_starter/test
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from robotics_starter.motion import (  # noqa: E402
    Pose, ShapeFollower, TeleopController, integrate, normalize_angle,
    twist_to_wheels, wheels_to_twist,
)


def test_normalize_angle():
    assert normalize_angle(0.0) == pytest.approx(0.0)
    assert normalize_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert normalize_angle(-3 * math.pi / 2) == pytest.approx(math.pi / 2)


def test_integrate_straight_line():
    p = integrate(Pose(0, 0, 0), v=1.0, w=0.0, dt=2.0)
    assert (p.x, p.y, p.theta) == pytest.approx((2.0, 0.0, 0.0))


def test_integrate_rotate_in_place():
    p = integrate(Pose(1, 1, 0), v=0.0, w=math.pi / 2, dt=1.0)
    assert (p.x, p.y, p.theta) == pytest.approx((1.0, 1.0, math.pi / 2))


def test_integrate_quarter_arc_exact():
    # Radius 1 m, quarter circle counter-clockwise from the origin heading +x.
    p = integrate(Pose(), v=1.0, w=1.0, dt=math.pi / 2)
    assert (p.x, p.y, p.theta) == pytest.approx((1.0, 1.0, math.pi / 2))


def test_wheel_conversion_round_trip():
    vl, vr = twist_to_wheels(0.5, 1.0, wheel_base=0.4)
    assert (vl, vr) == pytest.approx((0.3, 0.7))
    assert wheels_to_twist(vl, vr, 0.4) == pytest.approx((0.5, 1.0))


def test_teleop_steps_clamps_and_stops():
    t = TeleopController(linear_step=0.4, angular_step=1.0, max_linear=1.0, max_angular=1.5)
    assert t.handle_key('w') == (0.4, 0.0)
    assert t.handle_key('up') == (0.8, 0.0)
    assert t.handle_key('W') == (1.0, 0.0)          # clamped
    assert t.handle_key('a') == (1.0, 1.0)
    assert t.handle_key('left') == (1.0, 1.5)       # clamped
    assert t.handle_key('space') == (0.0, 0.0)
    assert t.handle_key('q') is None                # unused key


@pytest.mark.parametrize('loops', [1, 2])
def test_square_returns_to_start(loops):
    f = ShapeFollower('square', size=2.0, linear_speed=0.5, angular_speed=0.8, loops=loops)
    poses = f.simulate(dt=0.01)
    end = poses[-1]
    assert (end.x, end.y) == pytest.approx((0.0, 0.0), abs=1e-6)
    assert normalize_angle(end.theta) == pytest.approx(0.0, abs=1e-6)
    # It really is a 2 m square: opposite corner at (2, 2).
    assert max(p.x for p in poses) == pytest.approx(2.0, abs=1e-6)
    assert max(p.y for p in poses) == pytest.approx(2.0, abs=1e-6)


def test_circle_returns_to_start_with_right_radius():
    f = ShapeFollower('circle', size=1.5, linear_speed=0.5)
    poses = f.simulate(dt=0.05)
    assert (poses[-1].x, poses[-1].y) == pytest.approx((0.0, 0.0), abs=1e-6)
    # Circle centre is (0, r) for a CCW circle starting at the origin heading +x.
    for p in poses:
        assert math.hypot(p.x, p.y - 1.5) == pytest.approx(1.5, abs=1e-6)


def test_command_timeline():
    f = ShapeFollower('square', size=1.0, linear_speed=0.5, angular_speed=1.0)
    assert f.command(-1.0) == (0.0, 0.0, False)
    assert f.command(0.0) == (0.5, 0.0, False)       # forward segment, 2 s
    assert f.command(2.1) == (0.0, 1.0, False)       # turning
    assert f.command(f.total_duration + 0.1) == (0.0, 0.0, True)
    assert f.time_to_next_change(0.5) == pytest.approx(1.5)


def test_invalid_shape_rejected():
    with pytest.raises(ValueError):
        ShapeFollower('triangle')
    with pytest.raises(ValueError):
        ShapeFollower('square', size=0)
