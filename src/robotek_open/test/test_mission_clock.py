"""The end-of-round stop hold uses the field-equivalent clock."""

from robotek_open.open_mission_clock import mission_clock


def test_prefers_simulation_clock_when_available():
    assert mission_clock(True, 12.5, 99.0) == ('sim', 12.5)


def test_falls_back_to_wall_without_clock():
    assert mission_clock(False, 0.0, 99.0) == ('wall', 99.0)
