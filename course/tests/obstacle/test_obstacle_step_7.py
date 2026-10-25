"""Step 7: the mission and the strategy (my_obstacle/mission.py, strategy.py)."""

import pytest

from fieldsim import field_boxes, passes, run, scan
from my_obstacle.mission import Mission, make_params
from my_obstacle.strategy import Strategy
from robotek_course.obstacle_base import ObstacleStrategyBase


def test_the_strategy_is_a_course_strategy():
    strategy = Strategy()
    assert isinstance(strategy, ObstacleStrategyBase)
    assert strategy.mission.state == 'SETUP'


def test_each_scan_stamp_is_used_once():
    strategy = Strategy()
    s = scan(-0.25, -1.0, 0.0, field_boxes(), stamp=0.1)
    strategy.on_scan(s)
    strategy.on_scan(s)
    assert strategy.new_scan and strategy.scan_stamp == pytest.approx(0.1) and strategy.started


def test_unknown_parameters_are_refused():
    with pytest.raises(ValueError):
        make_params({'speed': 1.0})


@pytest.mark.parametrize('direction', ['ccw', 'cw'])
def test_setup_calibrates_and_finds_the_direction(direction):
    strategy = Strategy()
    _, strategy = run(strategy, [], direction, seconds=3.0, until=lambda s: s.mission.state == 'RUN')
    mission = strategy.mission
    assert mission.state == 'RUN' and mission.selected_direction == direction
    assert mission.gyro_bias == pytest.approx(0.006, abs=1e-6)
    assert mission.pose[:2] == pytest.approx((-0.25, -1.0), abs=0.02)


# Seats of the rules, world frame: (x, y, colour). The first straight of the round is the bottom one.
SIGNS_CCW = [(0.5, -0.9, 'red'), (1.1, 0.0, 'green'), (0.0, 1.1, 'red')]


@pytest.mark.parametrize('direction', ['ccw', 'cw'])
def test_it_passes_every_sign_on_its_side(direction):
    signs = SIGNS_CCW if direction == 'ccw' else [(-x, y, c) for x, y, c in SIGNS_CCW]
    track, strategy = run(Strategy(), signs, direction, seconds=40.0,
                          until=lambda s: s.mission.phi > 4.6)
    seen = passes(track, signs)
    assert len(seen) == 3
    for colour, side, distance in seen:
        assert side == ('left' if colour == 'red' else 'right')
        assert distance > 0.025 + 0.074 + 0.01                     # the sign's half and the car's half width


def test_it_stops_after_three_laps():
    track, strategy = run(Strategy(), [], 'ccw', seconds=150.0, until=lambda s: s.done)
    assert strategy.done and strategy.label == 'STOP'
    assert strategy.mission.phi == pytest.approx(24.7, abs=0.1)
    x, y = track[-1][1:3]
    assert -0.5 <= x <= 0.5 and y < -0.5                            # in the start section


def test_backing_up_at_start_only_with_a_sign_in_the_middle_seat():
    mission = Mission()
    mission.phi = 0.25
    mission.pillar = (-0.1, 0.5, 'red')
    mission.profile = [(0.35, -0.35), (0.65, -0.35)]
    assert mission._backup() > 0.3                                  # 0.25 m ahead: no room to change side
    mission.pillar = (-0.1, 1.0, 'red')
    assert mission._backup() == 0.0                                 # the end seat: a sign may stand behind


def test_it_stops_to_look_at_a_sign_of_unknown_colour():
    mission = Mission()
    mission.selected_direction, mission.phi = 'cw', 3.0
    mission.pillar = (0.1, 4.0, None)
    speed, steer, _ = mission._look(10.0, 0.0)
    assert speed < 0 and abs(steer) == pytest.approx(mission.p.steer_limit)
    mission.pillar = (0.1, 4.0, 'green')
    assert mission._look(10.1, 0.0) is None                          # the colour is in: carry on
