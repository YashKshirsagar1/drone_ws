"""Unit tests for the flight helpers. No simulator needed:

    pytest src/drone_bringup/test/test_flight.py
"""

import math
from types import SimpleNamespace

from drone_bringup.flight import Touchdown, clamp, world_to_body, yaw_from_quaternion


def close_to(expected, tol=1e-9):
    """Tiny stand-in so these comparisons read the same for scalars and pairs."""
    class Approx:
        def __eq__(self, other):
            if isinstance(expected, tuple):
                return all(abs(a - b) <= tol for a, b in zip(other, expected))
            return abs(other - expected) <= tol

        def __repr__(self):
            return f'approx({expected}, tol={tol})'
    return Approx()


def quat(yaw):
    return SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def test_clamp():
    assert clamp(5.0, 2.0) == 2.0
    assert clamp(-5.0, 2.0) == -2.0
    assert clamp(0.5, 2.0) == 0.5


def test_yaw_from_quaternion():
    for yaw in (0.0, 0.5, math.pi / 2, -math.pi / 2, 2.313):
        assert yaw_from_quaternion(quat(yaw)) == close_to(yaw)


def test_yaw_from_the_quaternion_the_sim_actually_reported():
    # Captured from /odom after yawing the drone in Gazebo.
    q = SimpleNamespace(x=-8.98e-19, y=1.08e-19, z=0.915251633217595,
                        w=0.4028826726139076)
    assert math.degrees(yaw_from_quaternion(q)) == close_to(132.5, tol=0.1)


def test_world_to_body_is_identity_at_zero_yaw():
    assert world_to_body(1.0, 0.0, 0.0) == close_to((1.0, 0.0))
    assert world_to_body(0.0, 1.0, 0.0) == close_to((0.0, 1.0))


def test_world_to_body_when_yawed():
    # Facing +90 deg (north), a target due east (world +x) is off to the right,
    # which is the body's -y.
    assert world_to_body(1.0, 0.0, math.pi / 2) == close_to((0.0, -1.0))
    # Facing 180 deg, a target at world +x is straight behind.
    assert world_to_body(1.0, 0.0, math.pi) == close_to((-1.0, 0.0))


def test_world_to_body_preserves_length():
    for yaw in (0.3, 1.2, -2.5):
        bx, by = world_to_body(3.0, -4.0, yaw)
        assert math.hypot(bx, by) == close_to(5.0)


def test_touchdown_ignores_a_real_descent():
    """Even the slowest rate the nodes command must not read as a touchdown."""
    for rate in (0.25, 0.7, 1.5):
        d, t, z = Touchdown(), 0.0, 4.0
        while z > 0.04:
            t, z = t + 0.05, max(0.04, z - rate * 0.05)
            assert not d.update(z, t), f'false touchdown while descending at {rate} m/s'


def test_touchdown_latches_once_the_drone_rests():
    d, t = Touchdown(settle=0.8), 0.0
    assert not d.update(0.04, t)
    t += 0.5
    assert not d.update(0.04, t), 'latched before the settle window elapsed'
    t += 0.4
    assert d.update(0.04, t), 'never latched after the settle window'


def test_touchdown_resets_for_reuse():
    d = Touchdown(settle=0.8)
    d.update(0.04, 0.0)
    assert d.update(0.04, 1.0)
    d.reset()
    assert not d.update(0.04, 1.0), 'reset did not clear the previous latch'


def close_to(expected, tol=1e-9):
    """Tiny stand-in so these comparisons read the same for scalars and pairs."""
    class Approx:
        def __eq__(self, other):
            if isinstance(expected, tuple):
                return all(abs(a - b) <= tol for a, b in zip(other, expected))
            return abs(other - expected) <= tol

        def __repr__(self):
            return f'approx({expected}, tol={tol})'
    return Approx()
