"""Obstacle v3 fixed parking (park_mode 1): parks on the kinematic model of the real car (it turns 1.114 x
tighter than its servo angle) from the approach, rolling on after every stop; lot parameters move the lot."""
import math
from types import SimpleNamespace

import pytest

from robotek_obstacle_v3.parking_search import move
from robotek_obstacle_v3 import park_fixed
from robotek_obstacle_v3.settings import make

GAIN = 1.114                 # real_v3: effective wheel angle / servo angle


def _drive(direction, overrides=None, roll=0.012, start_x=-0.3, steps=40000):
    p = make({'steer_limit': 0.3054, 'park': 1, 'park_after': 0, 'park_mode': 1, **(overrides or {})})
    mission = SimpleNamespace(p=p, selected_direction=direction)
    parking = park_fixed.FixedParking(mission, 0.06, 0.015)
    lot = parking.lot
    x, y, th = start_x if direction == 'ccw' else start_x - 0.7, -1.0, 0.0
    last_speed = last_steer = 0.0
    for i in range(steps):
        now = i * 0.02
        speed, steer = parking.step(now, x, y, th)
        if parking.state in ('PARKED', 'PARK_FAIL'):
            break
        if speed == 0.0 and abs(last_speed) > 0.01:     # not after the servo creep (below the motor deadband)
            x, y, th = move(x, y, th, math.atan(GAIN * math.tan(last_steer)), math.copysign(roll, last_speed))
        x, y, th = move(x, y, th, math.atan(GAIN * math.tan(steer)), speed * 0.02)
        last_speed, last_steer = speed, steer
        assert lot.free(x, y, th, steer, 0.0), f'contact at {x:.3f} {y:.3f} {math.degrees(th):.1f}'
    return parking, (x, y, th)


def test_parks_from_the_approach():
    parking, pose = _drive('ccw')
    assert parking.state == 'PARKED', (pose, parking.driven)
    assert parking.lot.parked(*pose, inside=0.0)


@pytest.mark.parametrize('direction, roll, first', [('cw', 0.012, 0.012), ('ccw', 0.03, 0.012)])
def test_ends_in_the_lot_without_contact(direction, roll, first):
    """No contact (asserted in _drive) and the car ends in the lot, deeper than its line. Whether it parks fully
    depends on the start and the car's lag (no lag here): the defaults were tuned in the lab (real_v3)."""
    parking, (x, y, th) = _drive(direction, {'park_fixed_roll': first}, roll=roll)
    assert parking.state in ('PARKED', 'PARK_FAIL')
    assert parking.lot.xa < x < parking.lot.xb and y < parking.lot.y1, (x, y, th)


def test_a_long_roll_does_not_touch():
    """8 cm after every stop (the speed floor of 8 nov): the guard keeps the car off the limits, even if it then
    cannot park (park_fixed_roll near the roll: the first stops have no measured roll yet)."""
    parking, _ = _drive('ccw', {'park_fixed_roll': 0.08}, roll=0.08)
    assert parking.state in ('PARKED', 'PARK_FAIL')


def test_lot_parameters_move_the_lot():
    lot = park_fixed.make_lot(1.0, make({'park_lot_length': 0.40, 'park_lot_depth': 0.22, 'park_lot_end': 0.50}))
    assert (round(lot.xa, 3), lot.xb, round(lot.y1 - lot.y0, 3)) == (0.10, 0.50, 0.22)
    cw = park_fixed.make_lot(-1.0, make({'park_lot_length': 0.40, 'park_lot_end': 0.50}))
    assert (cw.xa, round(cw.xb, 3)) == (-0.50, -0.10)
    parking, pose = _drive('ccw', {'park_lot_length': 0.40, 'park_lot_end': 0.55})
    assert parking.state == 'PARKED', (pose, parking.driven)


def test_default_mode_is_the_planner():
    assert make().park_mode == 0.0
