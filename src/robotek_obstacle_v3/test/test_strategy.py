"""The course wrapper of Obstacle v3: the sensor checks of the v3 node and one full round in the Obstacle lab."""
from pathlib import Path

from robotek_evaluation import obstacle_lab

from robotek_obstacle_v3.strategy import ROUND_LIMIT, Strategy


def _scan(now):
    return obstacle_lab.make_scan([1.0] * 306, now, -1.9, 0.0125)


def test_waits_for_every_sensor_after_start():
    strategy = Strategy()
    strategy.request_start()
    for i in range(20):                       # LiDAR and gyroscope, but no camera frame yet
        now = i * 0.02
        strategy.on_scan(_scan(now))
        strategy.on_imu(now, 0.0)
        assert strategy.control(now, 0.02) == (0.0, 0.0)
    assert strategy.round_start is None and strategy.mission.state == 'SETUP'
    strategy.on_signs([], 0.4)
    strategy.control(0.4, 0.02)
    assert strategy.round_start == 0.4 and strategy.label == 'SETUP'


def test_stale_camera_pauses_and_the_round_limit_ends():
    strategy = Strategy()
    strategy.request_start()
    strategy.on_scan(_scan(0.0))
    strategy.on_imu(0.0, 0.0)
    strategy.on_signs([], 0.0)
    strategy.control(0.0, 0.02)
    strategy.on_scan(_scan(0.5))
    strategy.on_imu(0.5, 0.0)
    assert 'camera' not in strategy.stale(0.5)             # 0.5 s late: still fresh (CAMERA_MAX_AGE 0.8)
    strategy.on_scan(_scan(0.9))
    strategy.on_imu(0.9, 0.0)
    assert strategy.control(0.9, 0.02) == (0.0, 0.0) and strategy.label == 'WAIT_SENSOR'
    late = ROUND_LIMIT + 1.0
    strategy.on_scan(_scan(late))
    strategy.on_imu(late, 0.0)
    strategy.on_signs([], late)
    assert strategy.control(late, 0.02) == (0.0, 0.0)
    assert strategy.label == 'FAULT' and strategy.done


def test_parking_ignores_a_stale_camera():
    strategy = Strategy(parking=True)
    strategy.request_start()
    strategy.on_scan(_scan(0.0))
    strategy.on_imu(0.0, 0.0)
    strategy.on_signs([], 0.0)
    strategy.control(0.0, 0.02)
    strategy.on_scan(_scan(0.9))
    strategy.on_imu(0.9, 0.0)
    assert set(strategy.stale(0.9)) == {'camera'}
    strategy.mission.state = 'PARK'
    assert strategy.stale(0.9) == {} and strategy.fresh(0.9)


def test_lab_round_with_the_lot(tmp_path: Path):
    field = obstacle_lab.make_field(tmp_path, 0, 'ccw', True)
    result = obstacle_lab.run_round(field, obstacle_lab.make_strategy('robotek_obstacle_v3.strategy:Strategy', True))
    assert result['valid'], result['faults']
    assert result['points'] == 55 and result['end'] == 'DONE'


def test_steer_watch_stops_when_full_lock_gives_no_turn():
    """11 nov 2026 23:59, corner 6: full left at 0.15 m/s, the gyro at -0.07 rad/s (0.4 expected). A fault after
    LAG + WINDOW s, a stop with the wheels centred for HOLD s; a good arc never trips it."""
    import math

    from robotek_obstacle_v3.steer_watch import HOLD, LAG, WINDOW, SteerWatch

    dt = 0.02
    for gyro, faults in ((0.30, 0), (-0.07, 1)):
        watch, first = SteerWatch(0.3054, 0.11883), None
        for k in range(int(1.5 / dt)):
            now = k * dt
            if watch.holding(now):
                continue
            if watch.update(now, dt, 0.15, 0.3054, gyro, 0.15 * dt) and first is None:
                first, hold_until = now, watch.hold_until
        assert watch.faults == faults
        if faults:
            assert LAG + WINDOW <= first < LAG + WINDOW + 0.1
            assert math.isclose(hold_until - first, HOLD)
            assert watch.last['ratio'] < 0 and math.isclose(watch.last['steer'], 17.5, abs_tol=0.1)


def test_front_recover_backs_up_on_the_side_that_makes_the_car_parallel():
    """12 nov 2026 00:26: nose 60 deg to the inside on the start straight (RECOVER_INSIDE 40 deg) -> back up on the other lock (+, three-point
    turn); a nose toward the outer wall (or straight) keeps the coach's rule (-steer_limit)."""
    import math

    from robotek_obstacle_v3.mission import Mission

    m = Mission({})
    m.phi = 0.5                                  # start straight: lane heading 0 in the mission frame
    limit = m.p.steer_limit
    assert m._recover_back_steer(math.radians(60.0)) == limit
    assert m._recover_back_steer(math.radians(30.0)) == -limit
    assert m._recover_back_steer(math.radians(-40.0)) == -limit


def test_a_lost_camera_holds_the_car_only_briefly():
    """12 nov 2026 01:05: the camera stopped at 14.5 s and the car waited in WAIT_SENSOR for good. Now a camera older
    than CAMERA_LOST no longer counts (scan and gyro still do); before Start it still does."""
    from robotek_obstacle_v3.strategy import CAMERA_LOST, Strategy

    s = Strategy()
    s.scan_stamp, s._imu_stamp, s.signs_stamp = 10.0, 10.0, 9.0
    assert 'camera' in s.stale(10.1)                  # before Start: the camera is needed
    s.started = True
    assert 'camera' in s.stale(10.1) and not s.camera_lost
    s.scan_stamp = s._imu_stamp = 9.0 + CAMERA_LOST + 0.1
    assert s.stale(9.0 + CAMERA_LOST + 0.2) == {} and s.camera_lost
    s.signs_stamp = 10.6
    s.stale(10.65)
    assert not s.camera_lost
