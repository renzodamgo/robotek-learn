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


def test_default_steps_in_order_with_their_lengths():
    p = make({'start_lot': 1.0})
    ex = lot_exit.LotExit(park_fixed.make_lot(1, p), 0.084, 10, 0.06, 0.3054)
    now, starts, last = 0.0, [], None
    while not ex.done and now < 30:
        speed, steer = ex.step(now, 0.0, 100)
        moving = (ex.name, speed > 0, steer > 0) if abs(speed) > 0.01 else None
        if moving and moving != last:
            starts.append([*moving, now])
        if moving and starts:
            starts[-1].append(now)
        last = moving
        now += 0.01
    assert ex.done
    assert [tuple(s[:3]) for s in starts] == [('1_LFWD', True, True), ('2_RBACK', False, False), ('3_LFWD', True, True),
                                              ('4_RFWD', True, False), ('5_RBACK', False, False), ('6_LFWD', True, True)]
    lengths = [round(s[-1] - s[3] + 0.01, 1) for s in starts]
    assert lengths == [0.5, 0.5, 2.0, 0.5, 1.0, 0.5]


def test_bad_steps_are_refused():
    assert [st[1:] for st in lot_exit.parse_steps('C+0.3 c-1')] == [(1.0, 0.0, 0.3), (-1.0, 0.0, 1.0)]
    for bad in ('', 'X+1', 'L*1', 'L+0', 'L+abc'):
        try:
            lot_exit.parse_steps(bad)
        except ValueError:
            continue
        raise AssertionError(bad)


def test_reverse_brakes_on_the_rear_sensor():
    p = make({'start_lot': 1.0})
    ex = lot_exit.LotExit(park_fixed.make_lot(1, p), 0.084, 10, 0.06, 0.3054, 'L+0.5 R-0.5 L+0.5')
    seen = run(ex, lambda e: 15 if e.name == '2_RBACK' else 100)
    assert ('BRAKE', '2_RBACK') in seen
    assert [e['braked'] for e in ex.log if e['event'] == 'lot_exit_pulse'] == [False, True, False]


def test_straight_steps_keep_the_wheels_centred_and_move_the_estimate():
    p = make({'start_lot': 1.0})
    ex = lot_exit.LotExit(park_fixed.make_lot(1, p), 0.084, 10, 0.2, 0.3054, 'C+0.5')
    x0 = ex.pose[0]
    now, steers = 0.0, set()
    while not ex.done and now < 5:
        speed, steer = ex.step(now, 0.0, 100)
        if abs(speed) > 0.01:
            steers.add(steer)
        now += 0.01
    assert steers == {0.0} and abs(ex.pose[0] - x0 - 0.1) < 0.01      # 0.5 s at 0.2 m/s


def test_front_recover_reverses_toward_parallel_to_the_outer_wall():
    from robotek_obstacle_v3.mission import Mission
    m = Mission({})
    m.selected_direction, m.pose = 'ccw', (0.3, -1.0, math.radians(30))   # bottom straight, nose 30 deg up
    steps = []
    for k in range(200):
        now = k * 0.02
        out = m._front_recovery(now, 0.125 if k == 0 else 2.0, 0.3)
        if out is None:
            break
        steps.append(out)
        if out[0] < 0:   # reversing at + steer turns the heading down
            x, y, th = m.pose
            m.pose = (x, y, th - 0.15 * 0.02 * math.tan(out[1]) / 0.11883)
    assert steps[0][0] == 0.003 and steps[0][1] > 0                      # brake first, wheels already turned
    assert any(s[0] < 0 for s in steps) and abs(m.pose[2]) < math.radians(4)   # reversed until parallel
