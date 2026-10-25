"""Open lab: world geometry, actuators, and a full standard round against the Gazebo reference."""
import math

import numpy as np
import pytest

from robotek_evaluation import open_lab as lab


@pytest.fixture(scope='module')
def world():
    return lab.World(lab.FIELDS / 'square100.sdf')


def test_raycast_reaches_the_island_and_the_outer_wall(world):
    south = world.raycast(0.0, -1.15, np.array([math.pi / 2]))     # island face at y = -0.50
    north = world.raycast(0.0, -1.15, np.array([-math.pi / 2]))    # outer wall at y = -1.50
    assert south[0] == pytest.approx(0.65, abs=1e-6)
    assert north[0] == pytest.approx(0.35, abs=1e-6)


def test_clearance_is_the_distance_from_the_outline_to_each_wall(world):
    start = world.clearances(lab.outline(0.0, -1.15, 0.0))
    assert start['outer_wall'] == pytest.approx(0.35 - 0.074, abs=0.01)
    assert start['inner_wall'] > 0.4
    touching = world.clearances(lab.outline(0.0, -1.43, 0.0))
    assert touching['outer_wall'] <= 0.0


def test_actuators_reach_the_real_speed_and_the_servo_is_rate_limited():
    actuators = lab.Actuators()
    dt = 0.01
    for i in range(300):
        actuators.command(i * dt, 1.28, 0.0)
        speed, _ = actuators.step(i * dt, dt)
    assert speed == pytest.approx(0.60, abs=0.02)         # the commanded 1.28 is 0.60 m/s on the car
    actuators.command(3.0, 1.28, 0.3)
    for i in range(10):
        actuators.step(3.0 + i * dt, dt)
    assert actuators.steer == 0.0                          # 0.10 s of dead time (measured in Gazebo)
    for i in range(10, 30):
        actuators.step(3.0 + i * dt, dt)
    assert 0.0 < actuators.steer < 0.28                    # then it moves at ~1.2 rad/s, not instantly


def test_the_car_turns_about_its_rear_axle():
    """Gazebo's base_link slides sideways by atan(0.0587 * yaw_rate / v) (0.14 rad at full lock)."""
    class Arc:
        label, turns, done, stop_label = 'FOLLOW', 0, False, None
        started = False

        def on_scan(self, scan):
            first, self.started = not self.started, True
            return first

        def on_imu(self, now, gyro_z):
            pass

        def on_camera(self, direction):
            pass

        def control(self, dt, age):
            return 1.28, 0.12
    result = lab.run_round(lab.FIELDS / 'square100.sdf', 'ccw', Arc, tmax=3.0, trace=True)
    rows = [r for r in result['rows'] if 2.0 <= r['sim_time'] <= 2.8]
    a, b = rows[0], rows[-1]
    dt = b['sim_time'] - a['sim_time']
    vx, vy = (b['x_gt'] - a['x_gt']) / dt, (b['y_gt'] - a['y_gt']) / dt
    yaw = (a['yaw_gt'] + b['yaw_gt']) / 2
    speed, omega = math.hypot(vx, vy), (b['yaw_gt'] - a['yaw_gt']) / dt
    sideslip = math.atan2(-vx * math.sin(yaw) + vy * math.cos(yaw), vx * math.cos(yaw) + vy * math.sin(yaw))
    assert omega > 0.3
    assert sideslip == pytest.approx(math.atan(0.0587 * omega / speed), abs=0.01)
