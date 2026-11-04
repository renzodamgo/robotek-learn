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
