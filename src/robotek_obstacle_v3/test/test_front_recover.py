"""The v3 front recover: a head-on obstacle fires it, a wall the car can still steer round does not."""
import math
import types

import numpy as np

from robotek_obstacle_v3.car import LIDAR_OFFSET_X, OUTLINE_FRONT
from robotek_obstacle_v3.mission import Mission


def _scan(points):
    """A 1 deg scan (base_link points -> ranges from the LiDAR)."""
    ranges = np.full(360, math.inf)
    for x, y in points:
        dx = x - LIDAR_OFFSET_X
        index = int(round(math.degrees(math.atan2(y, dx)))) % 360
        ranges[index] = min(ranges[index], math.hypot(dx, y))
    return types.SimpleNamespace(ranges=list(ranges), angle_min=0.0, angle_increment=math.radians(1.0),
                                 range_min=0.02, range_max=12.0)


def _mission(points):
    mission = Mission()
    mission.selected_direction = 'ccw'
    mission.pose = (0.0, -1.0, 0.0)
    mission._scan = _scan(points)
    return mission


def test_a_sign_straight_ahead_fires_the_recover():
    sign = [(OUTLINE_FRONT + 0.10, y) for y in np.linspace(-0.025, 0.025, 11)]
    mission = _mission(sign)
    speed, steer, state = mission._front_recovery(0.0, 0.2, 0.2, 0.0)
    assert mission.brake and mission.events[-1]['event'] == 'front_recover'
    assert steer == -mission.p.steer_limit            # CCW: the reverse is at full right


def test_a_wall_the_car_can_steer_round_does_not():
    # The outer wall on the left, the car 15 deg into it with the nose corner ~4 cm away: full right clears it.
    wall = [(x, 0.15 - x * math.tan(math.radians(15.0))) for x in np.linspace(0.0, 0.8, 200)]
    mission = _mission(wall)
    assert mission._in_path(0.0) is not None          # straight on it would touch
    assert mission._front_recovery(0.0, 0.3, 0.2, 0.0) is None
    assert not mission.brake and not mission.events
