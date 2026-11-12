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
        mission.step(DT, Scene(left=0.3, right=0.75, front_axis=front), gyro, camera)
    return mission


def test_it_waits_for_start_standing_still():
    mission = MiniMission()
    assert mission.label == 'WAIT_START'
    assert mission.step(DT, CLEAR, 0.0) == (0.0, 0.0)
    mission.request_start()
    mission.step(DT, CLEAR, 0.0)
    assert mission.label == 'CALIBRATE'


def test_after_calibrating_it_waits_for_the_camera_and_then_drives():
    mission = started()
    assert mission.label == 'DETECT_DIRECTION' and mission.state == 'SETUP'
    assert mission.step(DT, CLEAR, 0.0, 0) == (0.0, 0.0)
    mission.step(DT, CLEAR, 0.0, +1)
    assert mission.direction == 'ccw' and mission.state == 'RUN' and mission.label == 'FOLLOW'


def test_blue_means_counter_clockwise_and_orange_clockwise():
    assert started(camera=+1).direction == 'ccw'
    assert started(camera=-1).direction == 'cw'


def test_a_fixed_direction_skips_the_camera():
    assert started('cw', camera=0).state == 'RUN'


def test_without_a_direction_for_15_seconds_it_stops_instead_of_guessing():
    mission = started()
    for _ in range(int(15.5 / DT)):
        mission.step(DT, CLEAR, 0.0, 0)
    assert mission.state == 'STOP' and mission.label == 'DIRECTION_FAULT'
    assert mission.step(DT, CLEAR, 0.0, +1) == (0.0, 0.0)


def test_on_a_straight_it_cruises_and_keeps_the_gap():
    mission = started(camera=+1)
    speed, steer = mission.step(DT, Scene(left=0.45, right=0.6, front_axis=1.6, island_end=math.inf), 0.0)
    assert speed == 1.28 and steer > 0                       # too far from the island: steer towards it
    speed, steer = mission.step(DT, Scene(left=0.12, right=0.9, front_axis=1.6, island_end=math.inf), 0.0)
    assert steer < 0


def test_a_corner_starts_only_when_the_wall_is_near_and_the_island_has_ended():
    mission = started(camera=+1)
    mission.step(DT, CLEAR, 0.0)                             # the straight begins with the wall 1.40 m ahead
    for _ in range(3):                                       # 0.4 m later the wall is near, but the island has not ended
        mission.step(DT, Scene(left=0.3, right=0.75, front_axis=1.0, island_end=math.inf), 0.0)
    assert mission.label == 'FOLLOW'
    for _ in range(3):                                       # now its end is seen close, in new scans
        mission.step(DT, Scene(left=0.3, right=0.75, front_axis=1.0, island_end=0.2), 0.0)
    assert mission.label == 'TURN'
    assert math.isclose(mission.goal, math.pi / 2)


def test_a_clockwise_corner_turns_the_other_way():
    mission = started(camera=-1)
    mission.step(DT, Scene(left=0.75, right=0.3, front_axis=1.40), 0.0)
    for _ in range(3):
        mission.step(DT, Scene(left=0.75, right=0.3, front_axis=1.0, island_end=0.2), 0.0)
    assert mission.label == 'TURN' and math.isclose(mission.goal, -math.pi / 2)


def test_twelve_corners_then_it_creeps_and_stops_where_it_started():
    mission = started(camera=+1, front=1.40)
    mission.completed = 12
    near_start = Scene(left=0.3, right=0.75, front_axis=1.60, island_end=math.inf)
    speed, _ = mission.step(DT, near_start, 0.0)
    assert mission.label == 'FINISH' and speed == 0.15       # the wall ahead is farther than at Start: keep creeping
    at_start = Scene(left=0.3, right=0.75, front_axis=1.40, island_end=math.inf)
    for _ in range(int(1.5 / DT)):
        speed, _ = mission.step(DT, at_start, 0.0)
    assert speed == 0.0 and mission.label == 'DONE' and mission.stop_reason == 'done'


def test_the_motor_does_not_restart_after_braking():
    mission = started(camera=+1, front=1.40)
    mission.completed = 12
    mission.step(DT, Scene(left=0.3, right=0.75, front_axis=1.40), 0.0)                 # braking begins
    speed, _ = mission.step(DT, Scene(left=0.3, right=0.75, front_axis=1.70), 0.0)        # a noisy reading
    assert speed == 0.0


def test_the_operator_can_stop_it_for_good():
    mission = started(camera=+1)
    mission.operator_stop()
    assert mission.label == 'OPERATOR_STOP' and mission.step(DT, CLEAR, 0.0) == (0.0, 0.0)


def test_the_heading_comes_from_the_gyroscope_without_the_bias():
    mission = started(camera=+1, gyro=0.02)                  # a still gyroscope with a 0.02 rad/s bias
    for _ in range(150):
        mission.step(DT, CLEAR, 0.02 + 0.5)                  # 3 s turning at 0.5 rad/s
    assert math.isclose(mission.yaw, 1.5, abs_tol=0.01)      # without the calibration it would read 1.56


def drive_to_a_corner(mission, sign=1):
    """Feed the scans that start a corner: 0.4 m driven, the wall near and the island's end seen twice."""
    left, right = (0.3, 0.75) if sign > 0 else (0.75, 0.3)
    mission.step(DT, Scene(left=left, right=right, front_axis=1.40), 0.0)
    for _ in range(2):
        mission.step(DT, Scene(left=left, right=right, front_axis=1.0, island_end=0.2), 0.0)
    assert mission.label == 'TURN'


def test_it_slows_down_before_every_corner():
    mission = started(camera=+1)
    far, _ = mission.step(DT, Scene(left=0.25, right=0.75, front_axis=2.0), 0.0)
    near, _ = mission.step(DT, Scene(left=0.25, right=0.75, front_axis=1.40), 0.0)
    at, _ = mission.step(DT, Scene(left=0.25, right=0.75, front_axis=1.30), 0.0)
    assert far == 1.28 and math.isclose(near, 1.12) and at == 1.0     # 1.0 from 0.15 m before the corner may start
    mission.completed = 7                                             # the same before any other corner
    assert mission.step(DT, Scene(left=0.25, right=0.75, front_axis=1.30), 0.0)[0] == 1.0


def test_a_corner_is_full_lock_at_the_corner_speed_until_it_is_nearly_done():
    mission = started(camera=+1)
    drive_to_a_corner(mission)
    speed, steer = mission.step(DT, Scene(left=1.6, right=0.75, front_axis=2.0), 0.0)
    assert speed == 1.0 and math.isclose(steer, 0.30718)
    mission.heading.yaw = math.pi / 2 - 0.25                          # 0.25 rad to go: inside the tolerance
    speed, steer = mission.step(DT, Scene(left=1.6, right=0.75, front_axis=2.0), 0.0)
    assert mission.label == 'FOLLOW' and mission.completed == 1 and 0.0 < steer < 0.30718


def test_after_a_corner_it_must_drive_again_before_the_next_one():
    mission = started(camera=+1)
    drive_to_a_corner(mission)
    mission.heading.yaw = math.pi / 2
    mission.step(DT, Scene(left=1.6, right=0.75, front_axis=1.1), 0.0)             # the corner ends
    for _ in range(5):                                                # open side and a wall near, but nothing driven yet
        mission.step(DT, Scene(left=1.6, right=0.75, front_axis=1.1), 0.0)
    assert mission.label == 'FOLLOW' and mission.completed == 1
    mission.step(DT, Scene(left=1.6, right=0.75, front_axis=0.8), 0.0)             # 0.3 m nearer
    assert mission.label == 'TURN'


def test_the_island_wall_corrects_the_gyroscope_heading_on_a_straight():
    mission = started(camera=+1)                                      # the straight began with the wall 1.40 m ahead
    mission.step(DT, Scene(left=0.25, right=0.75, front_axis=1.00, wall_heading=0.10), 0.0)
    assert math.isclose(mission.yaw, 0.03)                            # 0.3 of the 0.10 rad difference
    mission.step(DT, Scene(left=0.25, right=0.75, front_axis=0.95, wall_heading=0.10), 0.0)
    assert math.isclose(mission.yaw, 0.03 + 0.3 * 0.07)               # every new scan closes 0.3 of what is left


def test_the_correction_runs_once_per_scan_and_not_right_after_a_corner():
    mission = started(camera=+1)
    crooked = Scene(left=0.25, right=0.75, front_axis=1.40, wall_heading=0.10)
    for _ in range(5):                                                # nothing driven yet on this straight
        mission.step(DT, crooked, 0.0)
    assert mission.yaw == 0.0
    scene = Scene(left=0.25, right=0.75, front_axis=1.00, wall_heading=0.10)
    for _ in range(5):                                                # the same scan five ticks in a row
        mission.step(DT, scene, 0.0)
    assert math.isclose(mission.yaw, 0.03)


def test_a_wall_heading_far_from_the_gyroscope_is_a_bad_fit():
    mission = started(camera=+1)
    mission.step(DT, Scene(left=0.25, right=0.75, front_axis=1.00, wall_heading=0.50), 0.0)    # 29 degrees apart
    assert mission.yaw == 0.0


def test_the_final_brake_waits_until_the_car_is_straight():
    mission = started(camera=+1, front=1.40)
    mission.completed = 12
    mission.heading.yaw = -0.25                                       # still coming out of the last corner
    at_start = Scene(left=0.3, right=0.75, front_axis=1.40)
    speed, steer = mission.step(DT, at_start, 0.0)
    assert speed == 0.15 and steer > 0                                # keeps creeping while it straightens
    mission.heading.yaw = -0.05
    assert mission.step(DT, at_start, 0.0)[0] == 0.0
