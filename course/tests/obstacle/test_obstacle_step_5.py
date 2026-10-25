"""Step 5: the corners (my_obstacle/corner.py)."""
import itertools

import pytest

from my_obstacle import corner, plan

ROWS = [(0.1, 'green'), (0.1, 'red'), (-0.1, 'green'), (-0.1, 'red'), None]


@pytest.mark.parametrize('sign_a,sign_b', list(itertools.product(ROWS, ROWS)))
def test_every_corner_has_a_line_with_clearance(sign_a, sign_b):
    gap, d_a, d_b, radius = corner.plan(sign_a, sign_b, 0.0, 0.0)
    # The two crossings (inner green -> outer red and back) are the tightest: 3.4 and 4.0 cm.
    assert gap >= 0.03
    assert radius >= 0.5


def test_the_line_passes_both_signs_on_their_side():
    gap, d_a, d_b, radius = corner.plan((0.1, 'green'), (-0.1, 'red'), 0.0, 0.0)
    assert d_a > 0.1 and d_b < -0.1
    knots = corner.knots(d_a, d_b, radius)
    assert knots[0][1] == pytest.approx(d_a)
    assert knots[-1][1] == pytest.approx(d_b)
    assert all(b[0] > a[0] for a, b in zip(knots, knots[1:]))        # progress keeps growing


def test_a_wrong_side_is_negative_clearance():
    samples = corner.path(-0.2, -0.2, 0.6)                           # outer line past an inner green at the exit
    assert corner.clearance(samples, None, (0.1, 'green')) < 0.0


def test_merge_replaces_the_signs_in_a_corner_by_its_line():
    signs = plan.with_offsets({1.0: (0.1, 'green'), 2.0: (-0.1, 'red')}, 0.254)
    knots = corner.merge(plan.sign_knots(signs, 0.2), signs, 0.2, plan.HORIZON)
    inside = [k for k in knots if 1.0 <= k[0] <= 2.0]
    assert len(inside) > 20                                           # the dense line, not two holds
    assert knots == sorted(knots)


def test_lines_keep_off_the_parking_lot():
    signs = plan.with_offsets({1.0: (-0.1, 'red')}, 0.254)
    free = corner.corner_line(1, signs)
    lot = corner.corner_line(1, signs, low_a=plan.LOT_OFFSET)
    assert min(d for s, d in lot if s <= 1.0) >= plan.LOT_OFFSET - 1e-9
    assert min(d for s, d in free if s <= 1.0) < plan.LOT_OFFSET
