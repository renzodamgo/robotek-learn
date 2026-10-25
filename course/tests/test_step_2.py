import math

from robotek_course.minisim import STEER_LIMIT, MiniCar
from helpers import drive_straight
from my_open.heading import GyroHeading, heading_steer


def test_calibration_measures_the_bias():
    heading = GyroHeading()
    heading.calibrate([0.006, 0.005, 0.007, 0.006])
    assert math.isclose(heading.bias, 0.006)


def test_the_heading_is_the_integral_of_the_gyroscope_without_the_bias():
    heading = GyroHeading()
    heading.calibrate([0.01] * 10)
    for _ in range(100):                           # 1 s turning at 0.51 rad/s as read (0.5 real)
        heading.update(0.51, 0.01)
    assert math.isclose(heading.yaw, 0.5, abs_tol=1e-9)


def test_without_calibration_the_bias_becomes_a_drift():
    heading = GyroHeading()
    for _ in range(1000):                          # 10 s standing still, with a 0.006 rad/s bias
        heading.update(0.006, 0.01)
    assert math.isclose(heading.yaw, 0.06, abs_tol=1e-9)


def test_steering_points_towards_the_goal():
    assert heading_steer(0.5, 0.0) > 0 > heading_steer(-0.5, 0.0)
    assert heading_steer(0.2, 0.2) == 0.0
    assert math.isclose(heading_steer(0.1, 0.0), 0.08)                       # KP = 0.8


def test_steering_never_exceeds_full_lock_and_takes_the_short_way_round():
    assert heading_steer(2.0, 0.0) == STEER_LIMIT
    assert heading_steer(-2.0, 0.0) == -STEER_LIMIT
    assert heading_steer(3.0, -3.0) < 0            # from -3 rad to 3 rad the short way is 0.28 rad BACKWARDS, across +-pi


def test_the_car_comes_back_to_the_goal_heading():
    car = MiniCar(yaw=0.5, realistic=False)
    for _ in range(250):
        car.step(0.6, heading_steer(0.0, car.yaw), 0.02)
    assert abs(car.yaw) < 0.02


def test_calibration_is_what_keeps_the_car_straight():
    uncalibrated = abs(drive_straight(calibrated=False))
    calibrated = abs(drive_straight(calibrated=True))
    assert uncalibrated > 0.04                     # about 3 degrees off after 10 s
    assert calibrated < 0.01
