import math

from robotek_course.minisim import Scene
from my_open.mission import MiniMission

DT = 0.02
CLEAR = Scene(left=0.30, right=0.75, front_axis=1.40, island_end=math.inf)


def started(direction='auto', camera=0, front=1.40, gyro=0.0):
    """A mission that has seen Start, calibrated for 0.3 s and (when ``camera`` is not 0) chosen its direction."""
    mission = MiniMission(direction)
    mission.request_start()
    for _ in range(25):
        mission.step(DT, Scene(left=0.3, right=0.75, front_axis=front), gyro, 0.0, camera)
    return mission


def test_it_waits_for_start_standing_still():
    mission = MiniMission()
    assert mission.label == 'WAIT_START'
    assert mission.step(DT, CLEAR, 0.0, 0.0) == (0.0, 0.0)
    mission.request_start()
    mission.step(DT, CLEAR, 0.0, 0.0)
    assert mission.label == 'CALIBRATE'


def test_after_calibrating_it_waits_for_the_camera_and_then_drives():
    mission = started()
    assert mission.label == 'DETECT_DIRECTION' and mission.state == 'SETUP'
    assert mission.step(DT, CLEAR, 0.0, 0.0, 0) == (0.0, 0.0)
    mission.step(DT, CLEAR, 0.0, 0.0, +1)
    assert mission.direction == 'ccw' and mission.state == 'RUN' and mission.label == 'FOLLOW'


def test_blue_means_counter_clockwise_and_orange_clockwise():
    assert started(camera=+1).direction == 'ccw'
    assert started(camera=-1).direction == 'cw'


def test_a_fixed_direction_skips_the_camera():
    assert started('cw', camera=0).state == 'RUN'


def test_without_a_direction_for_15_seconds_it_stops_instead_of_guessing():
    mission = started()
    for _ in range(int(15.5 / DT)):
        mission.step(DT, CLEAR, 0.0, 0.0, 0)
    assert mission.state == 'STOP' and mission.label == 'DIRECTION_FAULT'
    assert mission.step(DT, CLEAR, 0.0, 0.0, +1) == (0.0, 0.0)


def test_on_a_straight_it_cruises_and_keeps_the_gap():
    mission = started(camera=+1)
    speed, steer = mission.step(DT, Scene(left=0.45, right=0.6, front_axis=1.6, island_end=math.inf), 0.0, 0.01)
    assert speed == 1.28 and steer > 0                       # too far from the island: steer towards it
    speed, steer = mission.step(DT, Scene(left=0.12, right=0.9, front_axis=1.6, island_end=math.inf), 0.0, 0.01)
    assert steer < 0


def test_a_corner_starts_only_when_the_wall_is_near_and_the_island_has_ended():
    mission = started(camera=+1)
    for _ in range(30):                                      # drive 0.6 m
        mission.step(DT, CLEAR, 0.0, 0.02)
    for _ in range(3):                                       # wall near but the island has not ended
        mission.step(DT, Scene(left=0.3, right=0.75, front_axis=1.0, island_end=math.inf), 0.0, 0.0)
    assert mission.label == 'FOLLOW'
    for _ in range(3):                                       # now its end is seen close, in new scans
        mission.step(DT, Scene(left=0.3, right=0.75, front_axis=1.0, island_end=0.2), 0.0, 0.0)
    assert mission.label == 'TURN'
    assert math.isclose(mission.goal, math.pi / 2)


def test_a_clockwise_corner_turns_the_other_way():
    mission = started(camera=-1)
    for _ in range(30):
        mission.step(DT, CLEAR, 0.0, 0.02)
    for _ in range(3):
        mission.step(DT, Scene(left=0.75, right=0.3, front_axis=1.0, island_end=0.2), 0.0, 0.0)
    assert mission.label == 'TURN' and math.isclose(mission.goal, -math.pi / 2)


def test_twelve_corners_then_it_creeps_and_stops_where_it_started():
    mission = started(camera=+1, front=1.40)
    mission.completed = 12
    near_start = Scene(left=0.3, right=0.75, front_axis=1.60, island_end=math.inf)
    speed, _ = mission.step(DT, near_start, 0.0, 0.0)
    assert mission.label == 'FINISH' and speed == 0.15       # the wall ahead is farther than at Start: keep creeping
    at_start = Scene(left=0.3, right=0.75, front_axis=1.40, island_end=math.inf)
    for _ in range(int(1.5 / DT)):
        speed, _ = mission.step(DT, at_start, 0.0, 0.0)
    assert speed == 0.0 and mission.label == 'DONE' and mission.stop_reason == 'done'


def test_the_motor_does_not_restart_after_braking():
    mission = started(camera=+1, front=1.40)
    mission.completed = 12
    mission.step(DT, Scene(left=0.3, right=0.75, front_axis=1.40), 0.0, 0.0)            # braking begins
    speed, _ = mission.step(DT, Scene(left=0.3, right=0.75, front_axis=1.70), 0.0, 0.0)   # a noisy reading
    assert speed == 0.0


def test_the_operator_can_stop_it_for_good():
    mission = started(camera=+1)
    mission.operator_stop()
    assert mission.label == 'OPERATOR_STOP' and mission.step(DT, CLEAR, 0.0, 0.0) == (0.0, 0.0)


def test_the_heading_comes_from_the_gyroscope_without_the_bias():
    mission = started(camera=+1, gyro=0.02)                  # a still gyroscope with a 0.02 rad/s bias
    for _ in range(150):
        mission.step(DT, CLEAR, 0.02 + 0.5, 0.0)             # 3 s turning at 0.5 rad/s
    assert math.isclose(mission.yaw, 1.5, abs_tol=0.01)      # without the calibration it would read 1.56
