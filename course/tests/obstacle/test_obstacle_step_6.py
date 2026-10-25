"""Step 6: following the line (my_obstacle/follow.py)."""
import math

import pytest

from robotek_common.vehicle import WHEELBASE
from my_obstacle import follow, lane


def test_reference_on_a_straight_with_an_offset():
    x, y, heading, curvature = follow.reference(lambda s: 0.1, 0.5)
    assert (x, y) == pytest.approx((0.0, -0.9))
    assert heading == pytest.approx(0.0, abs=1e-9) and curvature == pytest.approx(0.0, abs=1e-9)


def test_reference_in_a_corner_is_the_arc():
    _, _, heading, curvature = follow.reference(lambda s: 0.0, 1.5)
    assert heading == pytest.approx(math.pi / 4, abs=1e-6)
    assert curvature == pytest.approx(2.0, rel=0.02)                  # 1 / 0.5 m
    assert follow.reference(lambda s: 0.2, 1.5)[3] == pytest.approx(1 / 0.3, rel=0.02)


def test_on_the_line_the_steering_is_the_feed_forward():
    phi = 1.5
    x, y, heading, _ = follow.reference(lambda s: 0.0, phi)
    angle = follow.steer((x, y, heading), phi, 0.0, lambda s: 0.0, 1.5, 0.2, 0.307)
    assert angle == pytest.approx(math.atan(WHEELBASE * 2.0), abs=0.01)


def test_left_of_the_line_steers_right():
    assert follow.steer((0.0, -0.95, 0.0), 0.5, 0.0, lambda s: 0.0, 1.5, 0.2, 0.307) < 0.0
    assert follow.steer((0.0, -1.05, 0.0), 0.5, 0.0, lambda s: 0.0, 1.5, 0.2, 0.307) > 0.0
    assert abs(follow.steer((0.0, -1.5, 0.0), 0.5, 0.0, lambda s: 0.0, 1.5, 0.2, 0.307)) == pytest.approx(0.307)


def _drive(offset, phi0, start_lateral, metres, settle=1.0, speed=0.3, dt=0.02):
    """A bicycle with a 0.06 s servo delay following the line; returns the largest error after ``settle`` metres."""
    x, y, heading = lane.lane_pose(phi0)
    nx, ny = lane.lane_normal(phi0)
    x, y = x + start_lateral * nx, y + start_lateral * ny
    phi, worst, travelled, queue, rate = phi0, 0.0, 0.0, [0.0, 0.0, 0.0], 0.0
    while travelled < metres:
        command = follow.steer((x, y, heading), phi, rate, offset, 1.5, 0.2, 0.307)
        queue.append(command)
        wheel = queue.pop(0)
        rx, ry = x - WHEELBASE / 2 * math.cos(heading), y - WHEELBASE / 2 * math.sin(heading)
        rate = speed * math.tan(wheel) / WHEELBASE
        heading += rate * dt
        rx, ry = rx + speed * dt * math.cos(heading), ry + speed * dt * math.sin(heading)
        x, y = rx + WHEELBASE / 2 * math.cos(heading), ry + WHEELBASE / 2 * math.sin(heading)
        travelled += speed * dt
        phi = float(lane.unwrap_s(phi, lane.track_s(x, y)))
        if travelled > settle:
            px, py, h, _ = follow.reference(offset, phi)
            worst = max(worst, abs(-(x - px) * math.sin(h) + (y - py) * math.cos(h)))
    return worst


def test_it_settles_on_a_line_15_cm_to_the_side_within_the_straight():
    assert _drive(lambda s: 0.15, 0.0, 0.0, 0.95, settle=0.7) < 0.01     # 0.55 cm after 0.7 m


def test_it_follows_a_whole_lap_within_two_centimetres():
    assert _drive(lambda s: 0.0, 0.0, 0.0, 8.0) < 0.02
    assert _drive(lambda s: 0.1, 0.0, 0.1, 8.0) < 0.02
