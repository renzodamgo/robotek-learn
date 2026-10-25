import math

from helpers import drive_lane
from my_open.lane import GAIN, GAP, lane_steer, lateral_bias


def test_no_offset_at_the_wanted_gap():
    assert lateral_bias(GAP) == 0.0


def test_a_car_too_far_aims_towards_the_island_and_one_too_close_aims_away():
    assert lateral_bias(0.45) > 0 > lateral_bias(0.15)
    assert math.isclose(lateral_bias(0.45), GAIN * (0.45 - GAP))


def test_the_offset_is_limited():
    assert lateral_bias(5.0) == 0.4 and lateral_bias(-5.0) == -0.4


def test_the_island_on_the_right_mirrors_everything():
    assert lateral_bias(0.45, sign=-1) == -lateral_bias(0.45)
    assert lane_steer(0.0, 0.0, 0.45, sign=-1) == -lane_steer(0.0, 0.0, 0.45, sign=1)


def test_the_steering_adds_the_offset_to_the_goal_heading():
    assert lane_steer(0.0, 0.0, 0.45) > 0                    # too far: turn towards the island
    assert lane_steer(0.0, 0.0, GAP) == 0.0


def test_the_car_closes_in_on_the_wanted_gap_and_never_touches_the_wall():
    gaps = [gap for _, gap in drive_lane(start_gap=0.45, seconds=14.0)]
    assert gaps[-1] < 0.31                                    # 0.45 m at the start
    assert min(gaps) > 0.20                                   # never closer than the wanted 0.25 m by much
    assert all(a >= b - 1e-3 for a, b in zip(gaps, gaps[1:]))  # it only gets closer: no oscillation
