"""Obstacle v3 as a course strategy: ``ObstacleStrategyBase`` wired to the v3 ``Mission``.

The base class already does what the v3 node did with the sensors (LiDAR odometry aligned once per scan stamp,
the gyroscope integrated with the mission's bias, Start); ``update`` adds the three things of the v3 node that the
base class does not do, so the decisions are the same as in WRO-Strategies:

* the distance of a tick counts only once the wall localizer is ready (before that it is dropped);
* the mission steps only while the scan, the camera and the gyroscope are fresh (``SENSOR_MAX_AGE``); otherwise
  the car waits with speed 0 (``WAIT_SENSOR``). The camera's age is the age of the last ``on_signs`` call: the node
  (``drive.py``) calls it for every frame, with an empty list when the frame has no scan to pair with;
* 180 s after Start the round is over (``FAULT``), as the rules say.

The mission's final state STOP is reported as ``DONE`` (as the v3 node did; the LEDs and the reporter know it).
"""
import math

from robotek_course.obstacle_base import ObstacleStrategyBase

from .mission import Mission

SENSOR_MAX_AGE = 0.35   # s; older data pauses the mission
ROUND_LIMIT = 180.0     # s after Start (rules 9.2)


class Strategy(ObstacleStrategyBase):
    def __init__(self, direction='auto', parking=False, params=None, **overrides):
        super().__init__()
        values = {**dict(params or {}), **overrides}
        if parking:
            values.setdefault('park', 1.0)
        self.direction = direction            # the mission reads it from the walls; kept for the record
        self.mission = Mission(values)
        self.round_start = None               # first tick after Start with every sensor fresh
        self._imu_stamp = -math.inf

    def on_imu(self, now, gyro_z):
        super().on_imu(now, gyro_z)
        self._imu_stamp = now

    def fresh(self, now):
        """Scan, camera and gyroscope all younger than SENSOR_MAX_AGE (the v3 node's test)."""
        return all(0 <= now - t < SENSOR_MAX_AGE for t in (self.scan_stamp, self.signs_stamp, self._imu_stamp))

    def update(self, now, dt):
        distance = self.travelled if self.mission.loc.ready else 0.0
        if self.label in ('DONE', 'FAULT'):
            return 0.0, 0.0
        fresh = self.fresh(now)
        if self.round_start is None:
            if not fresh:
                return 0.0, 0.0
            self.round_start = now
        if now - self.round_start > ROUND_LIMIT:
            self.label, self.done = 'FAULT', True
            return 0.0, 0.0
        if not fresh:
            self.label = 'WAIT_SENSOR'
            return 0.0, 0.0
        self.mission.observe(self.signs, self.signs_stamp)
        speed, steer, state = self.mission.step(now, self.scan, self.scan_stamp, self.signs_stamp,
                                                self.yaw, self.gyro, distance)
        self.gyro_bias = self.mission.gyro_bias
        self.label = 'DONE' if state == 'STOP' else state
        self.done = state == 'STOP'
        parking = self.mission.parking
        self.parked = parking is not None and parking.state == 'PARKED'
        return speed, steer
