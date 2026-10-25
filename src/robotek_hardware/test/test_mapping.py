import math

import pytest

from robotek_hardware.mapping import (MotorCalibration, ServoCalibration, mask_scan, motor_pct, rate_limit,
                                      servo_for_wheel,
                                      rotate_scan_angles, servo_us, steer_from_twist)


def test_servo_matches_the_team_mapping():
    cal = ServoCalibration()
    assert servo_us(0., cal) == 1500.
    # Team: delta +10 deg (right) -> d = -10 -> 1500 - 10*500/29
    assert math.isclose(servo_us(math.radians(-10.), cal), 1500.-10.*500./29.)
    # Left (+steer) -> team delta -10 -> d = +10 -> 1500 + 10*550/29
    assert math.isclose(servo_us(math.radians(10.), cal), 1500.+10.*550./29.)


def test_servo_respects_the_25_deg_limit():
    cal = ServoCalibration()
    assert servo_us(math.radians(40.), cal) == servo_us(math.radians(25.), cal)


def test_steer_from_twist_inverts_the_bicycle_model_and_holds_when_stopped():
    steer = math.radians(20.)
    v = .5
    w = v*math.tan(steer)/.13
    assert math.isclose(steer_from_twist(v, w, .13, 0.), steer)
    assert math.isclose(steer_from_twist(-v, -w, .13, 0.), steer)   # reverse keeps the angle
    assert steer_from_twist(0., 0., .13, .3) == .3


def test_motor_percent_is_signed_with_a_deadband():
    cal = MotorCalibration(pct_per_mps=40., min_pct=12.)
    assert motor_pct(0., cal) == 0.
    assert motor_pct(.5, cal) == 32.
    assert motor_pct(-.5, cal) == -32.
    assert motor_pct(10., cal) == 100.


def test_scan_rotation_puts_the_lidar_front_at_zero():
    # Driver: 0 deg backwards, front at +180 deg -> after +pi the front ray is at 360 = 0 deg.
    lo, hi = rotate_scan_angles(0., 2*math.pi, math.pi)
    front_index_angle = math.pi+math.pi
    assert math.isclose(math.cos(front_index_angle), 1.)
    assert (lo, hi) == (math.pi, 3*math.pi)


def test_rate_limit_ramps_up_and_down_and_stops_before_reversing():
    v, dt = 0., .02
    for _ in range(10):                       # 0.2 s at 1 m/s^2 -> 0.2 m/s
        v = rate_limit(v, .5, 1., .8, dt)
    assert math.isclose(v, .2)
    v = .5
    for _ in range(25):                       # 0.5 s at 0.8 m/s^2 -> 0.1 m/s
        v = rate_limit(v, 0., 1., .8, dt)
    assert math.isclose(v, .1, abs_tol=1e-9)
    assert rate_limit(.05, -.3, 1., .8, dt) >= 0.   # reversing goes through 0 first
    assert rate_limit(.3, .1, 0., 0., dt) == .1      # no limit


def test_mask_scan_drops_the_blind_sector_and_chassis_returns():
    # 0 rad ahead, 90 deg steps: ahead, left, behind, right
    r = mask_scan([1., 1., 1., .05], 0., math.pi/2, fov_deg=110., min_valid=.12)
    assert r[0] == 1. and r[1] == 1. and math.isinf(r[2]) and math.isinf(r[3])
    assert mask_scan([.05, 2.], 0., 1., 0., 0.) == [.05, 2.]


def test_servo_for_wheel_inverts_the_measured_table_per_side():
    servo, left, right = [10., 25.], [8.7, 19.7], [8.6, 17.6]
    assert math.isclose(servo_for_wheel(8.7, servo, left, right), 10.)
    assert math.isclose(servo_for_wheel(-17.6, servo, left, right, 17.6), -25.)
    assert math.isclose(servo_for_wheel(17.6, servo, left, right, 17.6), 10.+(17.6-8.7)*15./11.)
    assert math.isclose(servo_for_wheel(30., servo, left, right, 17.6), servo_for_wheel(17.6, servo, left, right))
    assert servo_for_wheel(.2, [], [], []) == .2                    # no table: unchanged


def test_low_voltage_needs_consecutive_readings_and_ignores_glitches():
    from robotek_hardware.mapping import LowVoltage
    low = LowVoltage(min_v=6.6, confirm=3)
    assert not low.update(6.5)
    assert not low.update(6.5)
    assert not low.update(7.2)      # a sag under load recovers: the count restarts
    assert not low.update(6.5)
    assert not low.update(0.)       # board glitch: ignored, count kept
    assert not low.update(6.4)
    assert low.update(6.4)
    assert not low.update(7.4)      # recovered: the detector does not latch, battery_guard does


def test_low_voltage_disabled_with_zero():
    from robotek_hardware.mapping import LowVoltage
    low = LowVoltage(min_v=0.)
    assert not any(low.update(5.) for _ in range(10))


def test_axes_matrix_old_flat_build_keeps_the_gyro_z_as_yaw():
    from robotek_hardware.mapping import axes_matrix, remap_vector
    matrix = axes_matrix('+y', '-x', '+z')
    assert remap_vector((0., 0., 1.), matrix) == (0., 0., 1.)
    assert remap_vector((0., 1., 0.), matrix) == (1., 0., 0.)   # board +y is forward
    assert remap_vector((1., 0., 0.), matrix) == (0., -1., 0.)  # board +x is to the right


def test_axes_matrix_rejects_a_mirrored_frame():
    import pytest
    from robotek_hardware.mapping import axes_matrix
    with pytest.raises(ValueError):
        axes_matrix('+y', '+x', '+z')
    with pytest.raises(ValueError):
        axes_matrix('+x', '+x', '+z')
    with pytest.raises(ValueError):
        axes_matrix('+q', '+y', '+z')


def test_vertical_board_puts_the_yaw_rate_on_another_gyro_axis():
    from robotek_hardware.mapping import axes_matrix, remap_vector
    # Board standing on its long edge: board x forward, board y up, board z to the right.
    matrix = axes_matrix('+x', '-z', '+y')
    assert remap_vector((0., 0.5, 0.), matrix) == (0., 0., 0.5)   # yaw rate sits on board y
    assert remap_vector((0., 9.81, 0.), matrix) == (0., 0., 9.81)  # gravity on board y


def test_covariance_follows_the_axes():
    from robotek_hardware.mapping import axes_matrix, remap_covariance
    matrix = axes_matrix('+y', '-x', '+z')
    covariance = [1., 0., 0., 0., 2., 0., 0., 0., 3.]
    assert remap_covariance(covariance, matrix) == [2., 0., 0., 0., 1., 0., 0., 0., 3.]


def test_estimate_axes_from_rest_and_a_forward_push():
    from robotek_hardware.mapping import estimate_axes
    # Vertical board: gravity on board +y, push on board +x (board z points right).
    found = estimate_axes((0.1, 9.7, -0.2), (2.0, 9.7, -0.1))
    assert (found['up'], found['forward'], found['left']) == ('+y', '+x', '-z')
    assert max(found['error_deg'].values()) < 15.
    # Old flat build: gravity on +z, push on board +y -> left is board -x.
    found = estimate_axes((0.6, -0.5, 9.6), (0.6, 1.5, 9.6))
    assert (found['up'], found['forward'], found['left']) == ('+z', '+y', '-x')


def test_forward_burst_stops_when_the_acceleration_flips():
    pytest.importorskip('rclpy')   # imu_axes.py is a ROS node: needs the ROS environment (source /opt/ros/jazzy/setup.bash)
    from robotek_hardware.imu_axes import forward_burst
    samples = [(0., 0., 0.)]*3 + [(2., 0., 0.)]*4 + [(-2., 0., 0.)]*4
    assert forward_burst(samples) == [2., 0., 0.]
