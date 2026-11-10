"""Lot exit (start_lot): the direction from the LiDAR's sides and the coach's pulse sequence."""
import math
import types

import numpy as np

from robotek_obstacle_v3 import lot_exit, park_fixed
from robotek_obstacle_v3.settings import make


def scan_with_sides(left, right):
    angles = np.linspace(-math.pi, math.pi, 720, endpoint=False)
    ranges = np.full(angles.size, 2.0)
    ranges[np.abs(angles - math.pi / 2) < 0.3] = left
    ranges[np.abs(angles + math.pi / 2) < 0.3] = right
    return types.SimpleNamespace(ranges=ranges.tolist(), angle_min=-math.pi, angle_increment=angles[1] - angles[0],
                                 range_min=0.02)


def test_direction_from_the_near_side():
    sign, wall = lot_exit.direction_in_lot(scan_with_sides(1.1, 0.5))
    assert sign == 1 and abs(wall - 0.5) < 0.01                                  # wall on the right: CCW
    assert lot_exit.direction_in_lot(scan_with_sides(0.5, 1.1))[0] == -1          # wall on the left: CW
    sign, wall = lot_exit.direction_in_lot(scan_with_sides(1.1, math.inf))        # too near to be seen
    assert sign == 1 and abs(wall - (lot_exit.WALL_GAP + lot_exit.OUTLINE_HALF_WIDTH)) < 1e-9
    assert lot_exit.direction_in_lot(scan_with_sides(0.5, 0.45))[0] == 0          # unsure


def run(ex, rear):
    now, heading, seen = 0.0, 0.0, []
    while not ex.done and now < 20:
        speed, steer = ex.step(now, heading, rear(ex))
        if abs(speed) > 0.01:
            seen.append((ex.name, speed > 0, steer > 0))
            heading += speed * 0.02 * math.tan(steer) / 0.11883
        if ex.brake:
            seen.append(('BRAKE', ex.name))
        now += 0.02
    return seen


def test_pulses_in_order():
    p = make({'start_lot': 1.0})
    ex = lot_exit.LotExit(park_fixed.make_lot(1, p), 0.084, 10, 0.06, 0.3054, 0.5)
    seen = run(ex, lambda e: 100)
    assert ex.done
    assert list(dict.fromkeys(seen)) == [('LEFT_FORWARD', True, True), ('RIGHT_BACK', False, False),
                                         ('LEFT_FORWARD_2', True, True), ('RIGHT_FORWARD', True, False)]
    assert [e['pulse'] for e in ex.log if e['event'] == 'lot_exit_pulse'] == [
        'LEFT_FORWARD', 'RIGHT_BACK', 'LEFT_FORWARD_2', 'RIGHT_FORWARD']


def test_reverse_brakes_on_the_rear_sensor():
    p = make({'start_lot': 1.0})
    ex = lot_exit.LotExit(park_fixed.make_lot(1, p), 0.084, 10, 0.06, 0.3054, 0.5)
    seen = run(ex, lambda e: 15 if e.name == 'RIGHT_BACK' else 100)
    assert ('BRAKE', 'RIGHT_BACK') in seen
    assert [e['braked'] for e in ex.log if e['event'] == 'lot_exit_pulse'] == [False, True, False, False]
