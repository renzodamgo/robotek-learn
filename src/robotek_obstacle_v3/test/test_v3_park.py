"""Obstacle v3 parking: the fixed manoeuvre closed on targets parks on the kinematic model."""
import math
from types import SimpleNamespace

import pytest

from robotek_common.vehicle import OUTLINE_HALF_WIDTH, REAL_STEER_LIMIT
from robotek_obstacle_v3.parking_search import Lot, move
from robotek_obstacle_v3 import park


def _drive(direction, start_error=(0.0, 0.0, 0.0), steps=20000):
    """Run Parking from its planning pose on the bicycle model at 50 Hz, rolling park.COAST on after
    every stop as the motor does -> (state, pose, moves)."""
    mission = SimpleNamespace(p=SimpleNamespace(steer_limit=REAL_STEER_LIMIT), selected_direction=direction)
    parking = park.Parking(mission, 0.06, 0.015)
    lot = parking.lot
    x = lot.xb + park.AHEAD + start_error[0]
    y = lot.y1 + OUTLINE_HALF_WIDTH + park.GAP + start_error[1]
    th = start_error[2]
    parking.state, parking.since = 'PARK_PLAN', 0.0
    last_speed = last_steer = 0.0
    for i in range(steps):
        now = i * 0.02
        speed, steer = parking.step(now, x, y, th)
        if parking.state in ('PARKED', 'PARK_FAIL'):
            break
        if speed == 0.0 and last_speed:
            x, y, th = move(x, y, th, last_steer, math.copysign(park.COAST, last_speed))
        x, y, th = move(x, y, th, steer, speed * 0.02)
        last_speed, last_steer = speed, steer
        assert lot.free(x, y, th, steer, 0.0), f'contact at {x:.3f} {y:.3f} {math.degrees(th):.1f}'
    return parking.state, (x, y, th), parking.driven


def test_nominal_moves_have_targets():
    moves = park.nominal_moves(REAL_STEER_LIMIT)
    assert 8 <= len(moves) <= 20
    kinds = {target[0] for *_, target in moves}
    assert 'heading' in kinds and 'depth' in kinds


@pytest.mark.parametrize('direction', ['ccw', 'cw'])
def test_parks_from_the_planning_pose(direction):
    state, pose, moves = _drive(direction)
    assert state == 'PARKED', (pose, moves)


@pytest.mark.parametrize('error', [(0.01, 0.0, 0.0), (-0.01, 0.005, 0.0), (0.0, -0.005, math.radians(1.5))])
def test_parks_from_a_perturbed_start(error):
    state, pose, moves = _drive('ccw', error)
    assert state == 'PARKED', (pose, moves)
