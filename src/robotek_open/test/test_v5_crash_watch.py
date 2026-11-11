import json
import math

from robotek_open.v5.crash_watch import CrashWatch
from robotek_open.v5.mission import OpenMission, Params


def scan(valid, total=504):
    return {'angle_min': 0.0, 'angle_increment': 2 * math.pi / total, 'range_min': 0.02, 'range_max': 25.0,
            'ranges': [1.0] * valid + [math.inf] * (total - valid)}


def test_blind_run_triggers_only_while_driving_and_after_blind_s():
    w = CrashWatch(blind_s=0.4)
    for i in range(10):                       # parked against a wall: not driving
        assert w.scan(i * 0.1, scan(50), moving=False) is None
    assert w.scan(1.0, scan(50), moving=True) is None
    assert w.scan(1.3, scan(50), moving=True) is None
    assert 'blind LiDAR' in w.scan(1.45, scan(50), moving=True)


def test_hysteresis_scans_between_min_and_clear_do_not_end_the_run():
    w = CrashWatch(min_rays=120, clear_rays=180, blind_s=0.4)
    w.scan(0.0, scan(60), True)
    w.scan(0.2, scan(150), True)              # 120-180: still blind
    assert w.scan(0.4, scan(70), True) is not None


def test_a_clear_scan_ends_the_run_and_normal_driving_never_triggers():
    w = CrashWatch(blind_s=0.4)
    w.scan(0.0, scan(60), True)
    w.scan(0.3, scan(290), True)
    assert w.scan(0.6, scan(60), True) is None
    w = CrashWatch()
    for i in range(100):
        assert w.scan(i * 0.1, scan(151 if i % 7 == 0 else 290), True) is None


def test_gyro_jump():
    w = CrashWatch(gyro_jump=3.0)
    assert w.tick(0.00, {}, -1.7, True) is None
    assert w.tick(0.05, {}, 0.5, True) is None    # 2.2 rad/s: a normal corner
    assert 'gyro jump' in w.tick(0.08, {}, 1.4, True)
    assert CrashWatch(gyro_jump=0.0).tick(0.0, {}, 9.0, True) is None


def test_window_keeps_the_last_seconds_and_dumps_json(tmp_path):
    w = CrashWatch(window_s=5.0)
    for i in range(100):
        w.tick(i * 0.1, {'v_cmd': 0.8}, 0.0, True)
        w.scan(i * 0.1, scan(290), True)
    assert w.ticks[0]['t'] >= 9.9 - 5.0 - 1e-9 and len(w.scans) <= 51
    data = json.load(open(w.dump(str(tmp_path / 'crash.json'), {'direction': 'ccw'})))
    assert data['direction'] == 'ccw' and data['scans'][-1]['valid'] == 290 and data['scans'][-1]['ranges'][-1] is None


def test_mission_crash_label():
    m = OpenMission(Params())
    m.crash_stop()
    assert m.label == 'CRASH' and m.step(0.02, None, 0.0, 0.0) == (0.0, 0.0)
