"""Step 8: parallel parking (my_obstacle/park.py)."""
import math
import time
from types import SimpleNamespace

import pytest

from robotek_common.vehicle import LENGTH, OUTLINE_HALF_WIDTH
from my_obstacle import park


def test_the_lot_of_the_rules():
    lot = park.Lot(1.0)
    assert lot.xb - lot.xa == pytest.approx(1.5 * LENGTH)          # 1.5 car lengths: 0.35 m
    assert (lot.y0, lot.y1) == pytest.approx((-1.5, -1.3))          # 20 cm against the outer wall
    cw = park.Lot(-1.0)
    assert (cw.xa, cw.xb) == pytest.approx((-lot.xb, -lot.xa))      # CW: the same lot, mirrored


def test_a_car_inside_and_parallel_is_parked():
    lot = park.Lot(1.0)
    middle = (lot.xa + lot.xb) / 2
    assert lot.parked(middle, -1.4, 0.0, inside=0.005)
    assert not lot.parked(middle, -1.4, math.radians(10))           # not parallel
    assert not lot.parked(middle, -1.2, 0.0)                        # half out


def test_the_bicycle_moves_about_the_rear_axle():
    x, y, th = park.move(0.0, 0.0, 0.0, 0.0, 0.1)
    assert (x, y, th) == pytest.approx((0.1, 0.0, 0.0))
    x, y, th = park.move(0.0, 0.0, 0.0, 0.3, -0.1)                  # reversing with the wheels left
    assert th < 0.0 and x < 0.0


def test_a_plan_from_the_planning_pose():
    started = time.monotonic()
    path = park.nominal_plan(0.307)
    assert path and time.monotonic() - started < 30.0
    lot = park.Lot(1.0)
    pose = (lot.xb + park.AHEAD, lot.y1 + OUTLINE_HALF_WIDTH + park.GAP, 0.0)
    for steer, direction in path:
        pose = lot.arc(pose, steer, direction, 0.005)
        assert pose is not None                                     # never closer than 5 mm to a limit or the wall
    assert lot.parked(*pose, inside=park.GOAL_INSIDE)


def _drive(direction, seconds=90.0, dt=0.02):
    """The parking manoeuvre in ideal kinematics, from the end of the laps (start straight, lane centre)."""
    mission = SimpleNamespace(p=SimpleNamespace(steer_limit=0.307), selected_direction=direction)
    parking = park.Parking(mission, 0.06, 0.015)
    x, y, th = -0.5, -1.15, 0.0
    t = 0.0
    while t < seconds and parking.state not in ('PARKED', 'PARK_FAIL'):
        speed, steer = parking.step(t, x, y, th)
        x, y, th = park.move(x, y, th, steer, speed * dt)
        t += dt
        assert parking.lot.free(x, y, th, 0.0, 0.0)                 # no contact, ever
    return parking, (x, y, th), t


@pytest.mark.parametrize('direction', ['ccw', 'cw'])
def test_it_parks(direction):
    parking, pose, seconds = _drive(direction)
    assert parking.state == 'PARKED', (parking.state, pose)
    assert parking.lot.parked(*pose, inside=0.0, parallel=math.radians(3.0))
    assert seconds < 90.0
