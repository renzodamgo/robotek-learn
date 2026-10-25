"""Step 4: the line, a side per sign (my_obstacle/plan.py)."""
import pytest

from robotek_common.vehicle import OUTLINE_HALF_WIDTH
from my_obstacle import plan

CLEARANCE = 0.18 + OUTLINE_HALF_WIDTH


def test_red_on_the_right_green_on_the_left():
    assert plan.side_offset(0.1, 'red', CLEARANCE) == pytest.approx(0.1 - CLEARANCE)
    assert plan.side_offset(-0.1, 'green', CLEARANCE) == pytest.approx(-0.1 + CLEARANCE)
    assert plan.side_offset(-0.1, 'red', 0.3) == pytest.approx(-plan.OFFSET_LIMIT)    # clipped: the wall is near
    assert plan.side_offset(0.1, 'green', 0.3) == pytest.approx(plan.OFFSET_LIMIT)


def test_unknown_colour_keeps_away_from_the_sign():
    assert plan.side_offset(0.1, None, CLEARANCE) == pytest.approx(-plan.UNKNOWN_OFFSET)
    assert plan.side_offset(-0.1, None, CLEARANCE) == pytest.approx(plan.UNKNOWN_OFFSET)


def test_the_offset_is_held_around_a_sign_with_ramps_between():
    signs = plan.with_offsets({1.0: (0.1, 'red'), 2.5: (-0.1, 'green')}, CLEARANCE)
    knots = sorted(plan.sign_knots(signs, 0.0))
    red, green = signs[1.0][2], signs[2.5][2]
    for s in (1.0 - plan.HOLD, 1.0, 1.0 + plan.HOLD):
        assert plan.offset_at(knots, s) == pytest.approx(red)
    assert plan.offset_at(knots, 2.5) == pytest.approx(green)
    middle = (1.0 + plan.HOLD + 2.5 - plan.HOLD) / 2
    assert plan.offset_at(knots, middle) == pytest.approx((red + green) / 2)
    assert plan.offset_at(knots, 0.0) == pytest.approx(red)                 # constant before the first knot
    assert plan.offset_at([], 3.0) == 0.0


def test_a_lookout_before_an_unknown_corner_exit_sign():
    signs = plan.with_offsets({4.0: (0.1, None)}, CLEARANCE)
    assert plan.lookouts(signs, 2.0) == [(2.5, plan.LOOKOUT)]
    assert plan.lookouts(plan.with_offsets({4.0: (-0.1, None)}, CLEARANCE), 2.0) == [(2.5, 0.0)]
    assert plan.lookouts(plan.with_offsets({5.0: (0.1, None)}, CLEARANCE), 3.0) == []         # a corner entry
    busy = plan.with_offsets({4.0: (0.1, None), 2.5: (0.1, 'red')}, CLEARANCE)
    assert plan.lookouts(busy, 2.0) == []                                                     # a sign is there


def test_the_lot_zone():
    assert plan.lot_zone(0.8, 'ccw') and not plan.lot_zone(0.2, 'ccw')
    assert plan.lot_zone(0.2, 'cw') and plan.lot_zone(7.9, 'cw') and plan.lot_zone(16.3, 'cw')
