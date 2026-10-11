"""Lidar docking: distance to the wall behind the robot from the rear-right beams."""
import math

import pytest

pytest.importorskip('nav2_simple_commander')
from tunibot_delivery.delivery_manager import rear_wall_distance  # noqa: E402

# the robot's lidar: 290 beams over +-145 deg, 0.12-12 m
N, AMIN, AMAX = 290, -2.5307, 2.5307
INC = (AMAX - AMIN) / (N - 1)


def scan_of_wall(dist, yaw=0.0, person=None):
    """Ranges to a wall `dist` m behind the lidar, robot turned by `yaw`; optional person
    (x, y, radius) in the lidar frame."""
    ranges = []
    for i in range(N):
        a = AMIN + i * INC
        dx, dy = math.cos(a + yaw), math.sin(a + yaw)        # beam in the wall-aligned frame
        r = dist / -dx if dx < -1e-6 else float('inf')
        if person:
            px, py, pr = person
            bx, by = math.cos(a), math.sin(a)
            t = px * bx + py * by
            d2 = px * px + py * py - t * t
            if t > 0 and d2 < pr * pr:
                r = min(r, t - math.sqrt(pr * pr - d2))
        ranges.append(r if 0.12 < r < 12.0 else float('inf'))
    return ranges


def test_distance_of_a_straight_wall():
    for d in (1.6, 1.75, 2.2):
        assert abs(rear_wall_distance(scan_of_wall(d), AMIN, INC, 0.12, 12.0) - d) < 1e-3


def test_heading_error_does_not_bias_the_distance():
    # the docking heading tolerance is 0.03 rad: a plain projection would be ~3 cm off
    for yaw in (-0.05, -0.03, 0.03, 0.05):
        assert abs(rear_wall_distance(scan_of_wall(1.6, yaw), AMIN, INC, 0.12, 12.0) - 1.6) < 2e-3


def test_chair_leg_in_the_beams_is_rejected():
    ranges = scan_of_wall(1.6, person=(-1.2, -1.2, 0.03))
    assert abs(rear_wall_distance(ranges, AMIN, INC, 0.12, 12.0) - 1.6) < 3e-3


def test_person_hiding_the_wall_gives_no_reading():
    # most beams hit the person: no reliable wall, the manager falls back to AMCL
    assert rear_wall_distance(scan_of_wall(1.6, person=(-1.0, -0.9, 0.22)), AMIN, INC, 0.12, 12.0) is None


def test_no_wall_behind():
    assert rear_wall_distance([float('inf')] * N, AMIN, INC, 0.12, 12.0) is None
