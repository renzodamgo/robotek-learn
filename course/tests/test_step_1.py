import math

from robotek_course.minisim import STEER_LIMIT, MiniCar
from my_open.model import bicycle_step, turn_radius, yaw_rate


def test_no_steering_drives_straight():
    x, y, yaw = 0.0, 0.0, 0.0
    for _ in range(200):
        x, y, yaw = bicycle_step(x, y, yaw, 0.6, 0.0, 0.01)
    assert math.isclose(x, 1.2, abs_tol=1e-6) and abs(y) < 1e-9 and yaw == 0.0


def test_yaw_rate_of_the_real_car_at_full_lock():
    assert math.isclose(yaw_rate(0.6, STEER_LIMIT), 0.6 * math.tan(STEER_LIMIT) / 0.1174)
    assert 1.6 < yaw_rate(0.6, STEER_LIMIT) < 1.65
    assert yaw_rate(0.6, -STEER_LIMIT) == -yaw_rate(0.6, STEER_LIMIT)       # right turns are negative
    assert yaw_rate(0.0, STEER_LIMIT) == 0.0                                 # a car that does not move does not turn


def test_smallest_circle_of_the_real_car():
    assert math.isclose(turn_radius(STEER_LIMIT), 0.370, abs_tol=0.002)
    assert turn_radius(-STEER_LIMIT) == turn_radius(STEER_LIMIT)
    assert turn_radius(0.1) > turn_radius(0.3)                               # less steering, wider circle


def test_a_full_circle_returns_to_the_start():
    radius = turn_radius(STEER_LIMIT)
    x, y, yaw = 0.0, 0.0, 0.0
    for _ in range(round(2 * math.pi * radius / 0.6 / 0.005)):
        x, y, yaw = bicycle_step(x, y, yaw, 0.6, STEER_LIMIT, 0.005)
    assert math.hypot(x, y) < 0.03


def test_agrees_with_the_mini_car():
    car = MiniCar(realistic=False)
    x, y, yaw = 0.0, 0.0, 0.0
    for _ in range(100):
        car.step(0.6, 0.2, 0.01)
        x, y, yaw = bicycle_step(x, y, yaw, 0.6, 0.2, 0.01)
    assert math.isclose(car.x, x, abs_tol=1e-9) and math.isclose(car.y, y, abs_tol=1e-9) and math.isclose(car.yaw, yaw)
