"""Step 2: where the car is (my_obstacle/localizer.py)."""
import math

import numpy as np
import pytest

from fieldsim import field_boxes, scan
from my_obstacle.localizer import Localizer, direction_from_scan, scan_points

BOXES = field_boxes()


def test_scan_points_are_in_the_car_frame_and_mirrored_for_cw():
    s = scan(-0.25, -1.0, 0.0, BOXES)
    bx, by = scan_points(s, 1.0)
    mx, my = scan_points(s, -1.0)
    assert np.allclose(bx, mx) and np.allclose(by, -my)
    assert bx.min() > 0.128 - 0.6        # the LiDAR is 0.128 m ahead of base_link and sees +-110 deg


def test_the_direction_from_the_walls():
    sign, margin = direction_from_scan(scan(-0.25, -1.0, 0.0, BOXES))
    assert sign == 1.0 and margin > 0.10
    sign, margin = direction_from_scan(scan(0.25, -1.0, math.pi, BOXES))     # CW: the island on the right
    assert sign == -1.0 and margin > 0.10


@pytest.mark.parametrize('x0', [-0.3, -0.25, 0.0, 0.2])
def test_initialize_finds_the_car_on_the_start_straight(x0):
    loc = Localizer()
    loc.initialize(*scan_points(scan(x0, -1.02, 0.03, BOXES), 1.0), 0.0)
    assert loc.ready
    assert (loc.x, loc.y) == pytest.approx((x0, -1.02), abs=0.02)
    assert loc.th == pytest.approx(0.03, abs=0.02)


def test_cw_is_the_mirror():
    loc = Localizer()
    loc.initialize(*scan_points(scan(0.25, -1.0, math.pi, BOXES), -1.0), 0.0)
    assert (loc.x, loc.y, loc.th) == pytest.approx((-0.25, -1.0, 0.0), abs=0.02)


def test_predict_moves_along_the_mean_heading():
    loc = Localizer()
    loc.predict(1.0, 0.2)
    assert (loc.x, loc.y, loc.th) == pytest.approx((math.cos(0.1), math.sin(0.1), 0.2))


def test_correct_pulls_a_wrong_guess_onto_the_walls():
    loc = Localizer()
    loc.x, loc.y, loc.th = 0.05, -0.96, 0.04          # 5 cm and 2 degrees off
    bx, by = scan_points(scan(0.0, -1.0, 0.0, BOXES), 1.0)
    for _ in range(5):
        assert loc.correct(bx, by, 0.5)
    assert (loc.x, loc.y, loc.th) == pytest.approx((0.0, -1.0, 0.0), abs=0.01)
    assert loc.rms < 0.005
