"""The node passes the round's parking to the mission (a default park=0 parameter must not cancel it)."""
import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.parameter import Parameter  # noqa: E402

from robotek_obstacle_v3 import drive  # noqa: E402


@pytest.mark.parametrize('overrides, park, clearance', [
    ([Parameter('parking', value=True)], 1.0, 0.18),
    ([Parameter('parking', value=False)], 0.0, 0.18),
    ([Parameter('parking', value=True), Parameter('pillar_clearance', value=0.2)], 1.0, 0.2),
])
def test_parking_and_parameters_reach_the_mission(monkeypatch, overrides, park, clearance):
    original = drive.Node.__init__

    def init(self, name, **kwargs):
        original(self, name, parameter_overrides=overrides, **kwargs)
    monkeypatch.setattr(drive.Node, '__init__', init)
    rclpy.init()
    try:
        node = drive.Drive()
        assert node.strategy.mission.p.park == park
        assert node.strategy.mission.p.pillar_clearance == clearance
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_the_round_is_recorded_like_the_open(monkeypatch, tmp_path):
    """telemetry.csv, events.jsonl (start, state changes, stop), the scans and params.yaml in the run folder, with a
    car-layout scan (504 rays, 0..2 pi) and the time since Start (12 nov 2026)."""
    import csv
    import gzip
    import json
    import math

    from builtin_interfaces.msg import Time
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import Bool

    folder = tmp_path / 'run'
    overrides = [Parameter('telemetry', value=str(folder)), Parameter('telemetry_every', value=1)]
    original = drive.Node.__init__

    def init(self, name, **kwargs):
        original(self, name, parameter_overrides=overrides, **kwargs)
    monkeypatch.setattr(drive.Node, '__init__', init)
    rclpy.init()
    try:
        node = drive.Drive()
        now = node.now()
        stamp = Time(sec=int(now), nanosec=int((now % 1) * 1e9))
        scan = LaserScan(angle_min=0.0, angle_increment=2 * math.pi / 504, range_min=0.05, range_max=12.0)
        scan.header.stamp = stamp
        scan.ranges = [1.0] * 504
        imu = Imu()
        imu.header.stamp = stamp
        node.on_scan(scan)
        node.on_imu(imu)
        node.on_start(Bool(data=True))
        for _ in range(4):
            node.control()
        assert node.t_start is not None
        node.rec.event('stop', node._t())
        node.rec.close()
        node.destroy_node()
    finally:
        rclpy.shutdown()
    rows = list(csv.DictReader((folder / 'telemetry.csv').open()))
    assert len(rows) == 4 and rows[0]['state'] and float(rows[0]['front']) == pytest.approx(1.0)
    events = [json.loads(line)['event'] for line in (folder / 'events.jsonl').open()]
    assert events[0] == 'start' and ('state' in events or 'wait_sensor' in events)
    assert events[-1] == 'stop'
    assert len(gzip.open(folder / 'telemetry.csv.scans.jsonl.gz', 'rt').readlines()) == 1
    assert 'changed:' in (folder / 'params.yaml').read_text()


def test_sections_signs_and_round_are_logged(monkeypatch, tmp_path):
    """Each section with its time, a sign seen and passed (side and OK / WRONG SIDE), the laps and the ROUND event."""
    import json

    folder = tmp_path / 'run'
    overrides = [Parameter('telemetry', value=str(folder))]
    original = drive.Node.__init__

    def init(self, name, **kwargs):
        original(self, name, parameter_overrides=overrides, **kwargs)
    monkeypatch.setattr(drive.Node, '__init__', init)
    rclpy.init()
    try:
        node = drive.Drive()
        mission = node.strategy.mission
        mission.state, mission.selected_direction = 'RUN', 'ccw'
        mission._seats = {1: [-0.1, {'red': 5}, 4]}        # seat 0.5 of straight 1, outer row, red
        clock = [0.0]
        monkeypatch.setattr(node, 'now', lambda: clock[0])
        node.t_start = 0.0
        from robotek_obstacle_v3 import scene
        for k in range(0, 801):                              # 8 units = one lap, 0.01 units a tick
            clock[0] = k * 0.05
            mission.phi = k * 0.01
            px, py, _ = scene.lane_pose(mission.phi)
            nx, ny = scene.lane_normal(mission.phi)
            lateral = -0.1 - 0.3 if 0.3 < mission.phi < 0.8 else 0.0   # right of the red sign
            mission.pose = (px + lateral * nx, py + lateral * ny, 0.0)
            node._log_progress(mission)
        node._log_round('STOPPED')
        node.rec.close()
        node.destroy_node()
    finally:
        rclpy.shutdown()
    events = [json.loads(line) for line in (folder / 'events.jsonl').open()]
    names = [e['event'] for e in events]
    assert names.count('section') == 8 and names.count('lap') == 1
    passed = [e for e in events if e['event'] == 'sign_passed']
    assert len(passed) == 1 and passed[0]['ok'] is True and passed[0]['side'] == 'outer'
    assert 'sign_seen' in names and events[-1]['event'] == 'round' and events[-1]['end'] == 'STOPPED'
