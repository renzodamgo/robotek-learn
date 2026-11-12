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
