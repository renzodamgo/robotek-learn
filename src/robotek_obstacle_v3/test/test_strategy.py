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
    assert strategy.control(0.5, 0.02) == (0.0, 0.0) and strategy.label == 'WAIT_SENSOR'
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
    strategy.on_scan(_scan(0.5))
    strategy.on_imu(0.5, 0.0)
    assert set(strategy.stale(0.5)) == {'camera'}
    strategy.mission.state = 'PARK'
    assert strategy.stale(0.5) == {} and strategy.fresh(0.5)


def test_lab_round_with_the_lot(tmp_path: Path):
    field = obstacle_lab.make_field(tmp_path, 0, 'ccw', True)
    result = obstacle_lab.run_round(field, obstacle_lab.make_strategy('robotek_obstacle_v3.strategy:Strategy', True))
    assert result['valid'], result['faults']
    assert result['points'] == 55 and result['end'] == 'DONE'
