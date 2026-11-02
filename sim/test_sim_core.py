"""Headless tests for the simulator core (no display / tkinter needed).

    python3 -m pytest sim
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))

from tk_sim import SimCore  # noqa: E402


def run(sim, seconds, dt=0.02):
    for _ in range(int(round(seconds / dt))):
        sim.step(dt)


@pytest.mark.parametrize('shape', ['square', 'circle'])
def test_shapes_close_with_frame_steps(shape):
    """With GUI-sized steps (20 ms) the shapes still return to the start."""
    sim = SimCore(shape_size=2.0)
    sim.start_shape(shape)
    run(sim, sim.follower.total_duration + 1.0)
    assert sim.mode == 'TELEOP' and sim.follower is None
    assert (sim.pose.x, sim.pose.y) == pytest.approx((0.0, 0.0), abs=1e-3)
    assert (sim.v, sim.w) == (0.0, 0.0)


def test_square_distance_travelled():
    sim = SimCore(shape_size=1.5)
    sim.start_shape('square')
    run(sim, 30.0, dt=0.033)
    assert sim.distance == pytest.approx(4 * 1.5, abs=1e-3)


def test_teleop_drives_and_key_cancels_shape():
    sim = SimCore()
    sim.key('w')
    sim.key('w')               # 0.2 m/s
    run(sim, 1.0)
    assert sim.pose.x == pytest.approx(0.2, abs=1e-6)

    sim.start_shape('circle')
    run(sim, 0.5)
    assert sim.key('shift_l') is False    # non-teleop key doesn't cancel
    assert sim.mode == 'CIRCLE'
    assert sim.key('x') is True           # teleop key cancels
    assert sim.mode == 'TELEOP' and (sim.teleop.v, sim.teleop.w) == (0.0, 0.0)


def test_reset_and_clear_trail():
    sim = SimCore()
    sim.key('w')
    sim.key('a')
    run(sim, 2.0)
    assert len(sim.trail) > 10
    sim.clear_trail()
    assert len(sim.trail) == 1
    sim.reset()
    assert (sim.pose.x, sim.pose.y, sim.pose.theta, sim.distance) == (0, 0, 0, 0)
    assert 'theta' in sim.readout() and math.isfinite(sim.time)
