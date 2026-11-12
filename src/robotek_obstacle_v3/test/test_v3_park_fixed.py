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


def _scan_of(points, pose):
    """Track-frame points -> base_link (what the LiDAR returns from the car at ``pose``)."""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return ([c * (px - x) + s * (py - y) for px, py in points], [-s * (px - x) + c * (py - y) for px, py in points])


def test_sense_finds_a_shifted_wall_and_lot():
    """park_fixed_sense: an outer wall 5 cm further out and a lot 3 cm further along than the map pose expects give
    the correction that puts them back (the car of 8 nov saw the wall at y -1.52..-1.57)."""
    p = make({'steer_limit': 0.3054, 'park': 1, 'park_after': 0, 'park_mode': 1, 'park_fixed_sense': 1})
    parking = park_fixed.FixedParking(SimpleNamespace(p=p, selected_direction='ccw'), 0.06, 0.015)
    lot = parking.lot
    wall = [(x / 100.0, lot.y0 - 0.05) for x in range(-50, 100)]
    # The faces toward the LiDAR (beyond both plates): each plate's +x face, 3 cm further along.
    plates = [(xp + 0.03, lot.y0 + 0.04 + 0.01 * k) for xp in (lot.xa, lot.xb + 0.02) for k in range(14)]
    pose = (lot.xb + 0.10, lot.y1 + 0.11, 0.0)
    parking.step(0.0, *pose)
    for _ in range(40):
        parking.observe(*_scan_of(wall + plates, pose))
    dx, dy, dth = parking.fix
    assert abs(dy - 0.05) < 0.005 and abs(dx + 0.03) < 0.01 and abs(dth) < 0.01, parking.fix
    assert parking.seen[0] == 40 and parking.seen[1] == 40


def test_sense_off_keeps_the_map_pose():
    p = make({'steer_limit': 0.3054, 'park': 1, 'park_after': 0, 'park_mode': 1})
    parking = park_fixed.FixedParking(SimpleNamespace(p=p, selected_direction='ccw'), 0.06, 0.015)
    parking.step(0.0, 0.5, -1.2, 0.0)
    parking.observe([0.0, 0.1], [-0.35, -0.35])
    assert parking.fix == [0.0, 0.0, 0.0]


@pytest.mark.parametrize('direction', ['ccw', 'cw'])
def test_partial_park_ends_after_the_entry_partly_in_the_lot(direction):
    """park_fixed_partial 1: approach, entry arc, straight into the lot, then stop (no straightening), without
    contact; the car's centre is past the lot's line and between its limits, angled (12 nov 2026)."""
    parking, (x, y, th) = _drive(direction, {'park_fixed_partial': 1})
    full, _ = _drive(direction)
    assert parking.state in ('PARKED', 'PARK_FAIL') and parking.log[-1].get('partial') is True
    assert parking.driven < full.driven
    assert parking.lot.xa < x < parking.lot.xb and y < parking.lot.y1, (x, y, math.degrees(th))
    assert abs(math.degrees(th)) > 10.0


def test_partial_park_then_the_finishing_pulses():
    """park_finish_steps after the partial park (12 nov 2026): the pulses run (park_pulse events), straighten the car
    and the round ends."""
    from robotek_obstacle_v3.settings import make as make_params

    def drive(steps):
        p = make_params({'steer_limit': 0.3054, 'park': 1, 'park_after': 0, 'park_mode': 1, 'park_fixed_partial': 1,
                         'park_pulse_speed': 0.25})
        mission = SimpleNamespace(p=p, selected_direction='ccw', park_finish_steps=steps)
        parking = park_fixed.FixedParking(mission, 0.25, 0.015)
        x, y, th, last = -0.3, -1.0, 0.0, (0.0, 0.0)
        for i in range(40000):
            speed, steer = parking.step(i * 0.02, x, y, th)
            if parking.state in ('PARKED', 'PARK_FAIL'):
                break
            if speed == 0.0 and abs(last[0]) > 0.01:
                x, y, th = move(x, y, th, math.atan(GAIN * math.tan(last[1])), math.copysign(0.012, last[0]))
            x, y, th = move(x, y, th, math.atan(GAIN * math.tan(steer)), speed * 0.02)
            last = (speed, steer)
        return parking, th

    plain, th_plain = drive('')
    finished, th_finished = drive('R+0.15 L-0.15')
    assert plain.state in ('PARKED', 'PARK_FAIL') and finished.state in ('PARKED', 'PARK_FAIL')
    assert [e['event'] for e in finished.log].count('park_pulse') == 2
    assert abs(th_finished) < abs(th_plain)


@pytest.mark.parametrize('shift', [-0.25, 0.0, 0.28])
def test_wide_sense_finds_the_plates_far_off(shift):
    """park_fixed_sense 2 (12 nov 2026, CW on the car: the pose 20-30 cm off along the straight): both plates
    `shift` m further along than the pose expects, seen from beside the lot, give fix[0] = -shift; mode 1 cannot
    (its band is +-6 cm). One plate alone (no pair) gives no along-track correction."""
    def plate_faces(xc, lidar_x):
        face = xc - 0.01 if xc > lidar_x else xc + 0.01
        return [(face, -1.5 + 0.04 + 0.01 * k) for k in range(14)]

    for mode in (1, 2):
        p = make({'steer_limit': 0.3054, 'park': 1, 'park_after': 0, 'park_mode': 1, 'park_fixed_sense': mode})
        parking = park_fixed.FixedParking(SimpleNamespace(p=p, selected_direction='ccw'), 0.06, 0.015)
        lot = parking.lot
        wall = [(x / 100.0, lot.y0) for x in range(-80, 100)]
        pose = (0.5 * (lot.xa + lot.xb) + shift, lot.y1 + 0.11, 0.0)    # beside the real lot
        lidar = pose[0] + 0.1
        plates = plate_faces(lot.xa - 0.01 + shift, lidar) + plate_faces(lot.xb + 0.01 + shift, lidar)
        parking.step(0.0, *pose)
        for _ in range(60):
            parking.observe(*_scan_of(wall + plates, pose))
        if mode == 2:
            assert abs(parking.fix[0] + shift) < 0.02, (shift, parking.fix)
        elif abs(shift) > 0.1:
            assert abs(parking.fix[0] + shift) > 0.1          # mode 1 does not get there

    p = make({'steer_limit': 0.3054, 'park': 1, 'park_after': 0, 'park_mode': 1, 'park_fixed_sense': 2})
    parking = park_fixed.FixedParking(SimpleNamespace(p=p, selected_direction='ccw'), 0.06, 0.015)
    lot = parking.lot
    pose = (lot.xb + 0.2, lot.y1 + 0.11, 0.0)
    parking.step(0.0, *pose)
    for _ in range(30):
        parking.observe(*_scan_of(plate_faces(lot.xb + 0.01 + 0.2, pose[0] + 0.1), pose))
    assert parking.fix[0] == 0.0
