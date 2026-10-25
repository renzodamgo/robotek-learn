"""Step 1: the field, the lane and progress (my_obstacle/lane.py)."""
import math
import random

import pytest

from my_obstacle import lane


def test_the_lane_starts_on_the_bottom_straight():
    x, y, heading = lane.lane_pose(0.0)
    assert (x, y) == pytest.approx((-0.5, -1.0)) and heading == pytest.approx(0.0)


def test_a_corner_is_a_quarter_circle_around_the_island_corner():
    x, y, heading = lane.lane_pose(1.5)
    assert math.hypot(x - 0.5, y + 0.5) == pytest.approx(0.5)
    assert heading == pytest.approx(math.pi / 4)
    assert lane.lane_pose(2.0) == pytest.approx((1.0, -0.5, math.pi / 2))
    x, y, heading = lane.lane_pose(4.0)
    assert (x, y) == pytest.approx((0.5, 1.0)) and math.cos(heading) == pytest.approx(-1.0)


def test_the_lane_closes_after_eight_units():
    a, b = lane.lane_pose(0.3), lane.lane_pose(8.3)
    assert a[:2] == pytest.approx(b[:2])
    assert lane.lane_pose(-0.5)[:2] == pytest.approx(lane.lane_pose(7.5)[:2])


def test_the_normal_points_to_the_island():
    assert lane.lane_normal(0.5) == pytest.approx((0.0, 1.0))
    assert lane.lane_normal(2.5) == pytest.approx((-1.0, 0.0))


def test_advance_counts_metres_at_the_local_scale():
    assert lane.advance(0.2, 0.5) == pytest.approx(0.7)
    assert lane.advance(1.0, lane.CORNER_UNITS) == pytest.approx(2.0)


def test_track_s_gives_progress_for_any_point():
    assert float(lane.track_s(0.0, -1.0)) == pytest.approx(0.5)
    assert float(lane.track_s(0.0, -1.3)) == pytest.approx(0.5)          # any lateral position of the straight
    assert float(lane.track_s(1.0, 0.0)) == pytest.approx(2.5)
    assert float(lane.track_s(1.2, -1.2)) == pytest.approx(1.5)          # the middle of the first corner


def test_unwrap_keeps_counting_laps():
    assert lane.unwrap_s(7.9, 0.1) == pytest.approx(8.1)
    assert lane.unwrap_s(16.2, 0.1) == pytest.approx(16.1)
    assert lane.unwrap_s(8.1, 7.9) == pytest.approx(7.9)


def test_the_twelve_seats():
    assert len(lane.SEATS) == 12
    assert {0.0, 0.5, 1.0, 2.0, 2.5, 3.0, 6.0, 7.0} <= set(lane.SEATS)
    assert lane.ROW == pytest.approx(0.1)


def test_progress_of_finds_the_nearest_lane_point_ahead():
    assert lane.progress_of(1.0, 0.0, 2.0) == pytest.approx(2.5, abs=0.02)
    assert lane.progress_of(0.0, -0.9, 0.0) == pytest.approx(0.5, abs=0.02)   # a sign of the inner row
    rng = random.Random(3)
    for _ in range(200):
        start = rng.uniform(0.0, 8.0)
        s = start + rng.uniform(0.2, 2.5)
        x, y, _ = lane.lane_pose(s)
        assert lane.progress_of(x, y, start) == pytest.approx(s, abs=0.011)
