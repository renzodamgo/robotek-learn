"""Open v5, layer 1: the mission on hand-made scenes (no simulator)."""
import math

import pytest

from robotek_common.vehicle import REAL_STEER_LIMIT
from robotek_open.v5.mission import CORNERS, OpenMission, Params
from robotek_open.v5.scene import Scene

DT = 0.02
BESIDE = Scene(left=0.45, right=0.55, front_axis=1.8)          # island on the left, far wall ahead


def run_setup(mission, scene=BESIDE, camera=0, seconds=0.5):
    mission.request_start()
    for _ in range(int(seconds / DT)):
        mission.step(DT, scene, 0.004, camera)


def ready(direction='ccw', **params):
    mission = OpenMission(Params(direction=direction, **params))
    run_setup(mission)
    assert mission.state == 'RUN'
    return mission


def test_setup_is_quiet_calibrates_the_gyro_and_remembers_the_start_range():
    mission = OpenMission(Params(direction='ccw'))
    assert mission.step(DT, BESIDE, 0.0) == (0.0, 0.0) and mission.state == 'SETUP'   # waits for Start
    run_setup(mission)
    assert mission.state == 'RUN'
    assert mission._bias == pytest.approx(0.004)
    assert mission._start_front == pytest.approx(1.8)


def test_the_camera_decides_the_direction_and_without_it_the_mission_stops_after_15_s():
    mission = OpenMission(Params(direction='auto'))
    run_setup(mission, camera=-1)
    assert mission.direction == 'cw' and mission.state == 'RUN'
    lost = OpenMission(Params(direction='auto'))
    run_setup(lost, seconds=16.0)
    assert lost.label == 'DIRECTION_FAULT'
    assert lost.step(DT, BESIDE, 0.0) == (0.0, 0.0)


def test_without_a_camera_decision_the_fallback_direction_is_taken_after_the_wait():
    mission = OpenMission(Params(direction='auto', direction_fallback='ccw'))
    run_setup(mission, seconds=0.9)
    assert mission.state == 'SETUP' and not mission.direction          # still waiting for the camera
    run_setup(mission, seconds=0.6)
    assert mission.direction == 'ccw' and mission.state == 'RUN' and mission.direction_fallback_used
    decided = OpenMission(Params(direction='auto', direction_fallback='ccw'))
    run_setup(decided, camera=-1)
    assert decided.direction == 'cw' and not decided.direction_fallback_used   # the camera still wins


def test_it_drives_straight_at_cruise_speed_and_holds_the_gap_to_the_island():
    mission = ready()
    speed, steer = mission.step(DT, Scene(left=0.45, right=0.55, front_axis=1.8), 0.004)
    assert speed == pytest.approx(1.28)
    assert steer > 0.0                      # 0.45 m from the island, 0.25 wanted: steer towards it (left, +)
    _, steer = mission.step(DT, Scene(left=0.12, right=0.9, front_axis=1.8), 0.004)
    assert steer < 0.0                      # too close: steer away


def test_a_turn_starts_when_the_front_wall_is_near_and_the_island_has_ended():
    mission = ready()
    mission.travelled = 1.0
    near_but_island_continues = Scene(left=0.45, right=0.55, front_axis=1.0, island_end=2.0)
    mission.step(DT, near_but_island_continues, 0.004)
    assert not mission.arc                                           # island still beside the car
    ended = Scene(left=2.4, right=0.55, front_axis=1.0)
    speed, steer = mission.step(DT, ended, 0.004)
    assert mission.arc and mission.goal == pytest.approx(math.pi / 2)
    assert steer > 0.1 and steer <= REAL_STEER_LIMIT                  # counter-clockwise: left


def test_the_island_end_must_be_seen_in_two_different_scans():
    mission = ready()
    mission.travelled = 1.0
    scan = Scene(left=0.45, right=0.55, front_axis=1.1, island_end=0.2)
    mission.step(DT, scan, 0.004)
    mission.step(DT, scan, 0.004)                               # the same scan again: one sighting
    assert not mission.arc
    mission.step(DT, Scene(left=0.45, right=0.55, front_axis=1.1, island_end=0.2), 0.004)
    assert mission.arc


def test_clockwise_turns_the_other_way():
    mission = ready('cw')
    mission.travelled = 1.0
    _, steer = mission.step(DT, Scene(left=0.55, right=2.4, front_axis=1.0), 0.004)
    assert mission.goal == pytest.approx(-math.pi / 2) and steer < -0.1


def finish_arc(mission, ended=Scene(left=2.4, right=0.55, front_axis=1.0)):
    """Start the next corner and complete it by turning the integrated yaw to the goal."""
    mission.travelled = 1.0
    mission.step(DT, ended, 0.004)
    mission.yaw = mission.goal
    mission.step(DT, BESIDE, 0.004)


def test_an_arc_ends_when_the_heading_reaches_the_goal_and_the_next_one_needs_some_distance():
    mission = ready()
    finish_arc(mission)
    assert not mission.arc and mission.completed == 1 and mission.travelled < 0.1
    mission.step(DT, Scene(left=2.4, right=0.55, front_axis=1.0), 0.004)
    assert not mission.arc                                           # ARM_DISTANCE not driven since the corner
    mission.travelled = 1.0
    mission.step(DT, Scene(left=2.4, right=0.55, front_axis=1.0), 0.004)
    assert mission.arc


def test_a_corner_is_driven_at_full_lock_until_turn_tolerance():
    mission = ready(turn_tolerance=0.3)
    mission.travelled = 1.0
    _, steer = mission.step(DT, Scene(left=2.4, right=0.55, front_axis=1.0), 0.004)
    assert mission.arc and steer == pytest.approx(REAL_STEER_LIMIT)
    mission.yaw = mission.goal - 0.29
    mission.step(DT, Scene(left=2.4, right=0.55, front_axis=1.0), 0.004)
    assert not mission.arc and mission.completed == 1


def test_the_car_brakes_towards_corner_speed_before_a_corner():
    mission = ready(corner_speed=0.6)
    far = mission.step(DT, Scene(left=0.45, right=0.55, front_axis=3.0), 0.004)[0]
    near = mission.step(DT, Scene(left=0.45, right=0.55, front_axis=1.3), 0.004)[0]
    assert far == pytest.approx(1.28) and 0.6 <= near < far


def test_between_scans_the_front_wall_is_dead_reckoned_with_the_commanded_speed():
    mission = ready()
    mission.travelled = 1.0
    scene = Scene(left=0.45, right=0.55, front_axis=1.5)
    speed, _ = mission.step(DT, scene, 0.004)
    for _ in range(10):                                   # no new scan for 0.2 s (the LD19 skips some)
        mission.step(DT, scene, 0.004)
    assert 1.5 - speed * 10 * DT <= mission.front < 1.5 - 0.1   # it brakes as the wall nears: a bit less
    mission.step(DT, Scene(left=0.45, right=0.55, front_axis=1.2), 0.004)
    assert mission.front == 1.2                           # a new scan resets it


def test_after_the_twelfth_corner_it_stops_where_the_front_range_matches_the_start():
    mission = ready()
    mission.completed = CORNERS
    assert mission.step(DT, Scene(left=0.45, right=0.55, front_axis=1.8 + 0.5), 0.004)[0] == pytest.approx(0.15)
    for _ in range(int(1.2 / DT)):
        speed, _ = mission.step(DT, Scene(left=0.45, right=0.55, front_axis=1.85), 0.004)
    assert speed == 0.0 and mission.label == 'DONE'


def test_the_operator_can_stop_it_for_good():
    mission = ready()
    mission.operator_stop()
    assert mission.label == 'OPERATOR_STOP' and mission.step(DT, BESIDE, 0.0) == (0.0, 0.0)


def corner_scans(mission, scene, n):
    for _ in range(n):
        mission.step(DT, Scene(**{f.name: getattr(scene, f.name) for f in scene.__dataclass_fields__.values()}),
                     0.004)


def test_a_wrong_camera_direction_is_corrected_when_the_other_side_opens_before_the_first_corner():
    mission = ready('cw')                                    # the camera said cw: the island is expected on the right
    mission.travelled = 1.0
    corner_scans(mission, Scene(left=5.5, right=0.34, front_axis=1.0), 2)
    assert mission.direction == 'cw' and not mission.arc    # two scans are not enough
    corner_scans(mission, Scene(left=5.5, right=0.34, front_axis=1.0), 1)
    assert mission.direction == 'ccw' and mission.direction_flipped
    mission.step(DT, Scene(left=5.5, right=0.34, front_axis=1.0), 0.004)
    assert mission.arc and mission.goal == pytest.approx(math.pi / 2)   # and it turns left


def test_the_direction_check_ignores_a_normal_corner_and_stops_after_the_first_one():
    mission = ready('ccw')
    mission.travelled = 1.0
    corner_scans(mission, Scene(left=2.4, right=0.55, front_axis=1.0), 5)    # island (left) ends: a normal corner
    assert mission.direction == 'ccw' and not mission.direction_flipped and mission.arc
    mission.completed = 1
    mission.arc = False
    corner_scans(mission, Scene(left=0.45, right=5.0, front_axis=1.8), 5)    # after corner 1: never flips
    assert mission.direction == 'ccw'


def test_wall_heading_pulls_the_gyro_yaw_towards_the_walls_on_a_straight():
    mission = ready('cw', wall_heading_gain=0.5)
    mission.travelled = 1.0
    mission.yaw = mission.goal                                  # the gyro believes it is straight
    for _ in range(10):                                         # the walls say 10 deg to the right
        mission.step(DT, Scene(left=0.55, right=0.45, front_axis=1.8, wall_heading=math.radians(-10)), 0.004)
    assert math.degrees(mission.yaw - mission.goal) == pytest.approx(-10, abs=0.1)


def test_wall_heading_is_off_by_default_and_ignores_big_differences():
    mission = ready('cw')
    mission.travelled = 1.0
    mission.step(DT, Scene(left=0.55, right=0.45, front_axis=1.8, wall_heading=math.radians(-10)), 0.004)
    assert mission.yaw == pytest.approx(mission.goal, abs=1e-3)
    on = ready('cw', wall_heading_gain=0.5)
    on.travelled = 1.0
    on.step(DT, Scene(left=0.55, right=0.45, front_axis=1.8, wall_heading=math.radians(-40)), 0.004)
    assert on.yaw == pytest.approx(on.goal, abs=1e-3)           # 40 deg: a bad fit, not drift
