"""The strategy (step 7): the course's base class wired to the mission.

``Strategy`` is what the node (``drive.py``), the Obstacle lab and the tests run. The base class
gives it the sensors (scan, integrated gyro, odometry distance, signs); ``update`` hands them to
the ``Mission`` and returns its speed and steering.
"""
from robotek_course.obstacle_base import ObstacleStrategyBase
from robotek_course.obstacle_vision import scan_clusters

from .mission import Mission


class Strategy(ObstacleStrategyBase):
    def __init__(self, direction='auto', parking=False, params=None):
        super().__init__()
        values = dict(params or {})
        if parking:
            values.setdefault('park', 1.0)
        self.direction = direction          # the mission reads it from the walls; kept for the record
        self.mission = Mission(values)
        self._clusters = []
        self._clusters_stamp = None

    def update(self, now, dt):
        if self.new_scan and self.scan_stamp != self._clusters_stamp:
            self._clusters, self._clusters_stamp = scan_clusters(self.scan), self.scan_stamp
        self.mission.observe(self.signs, self.signs_stamp)
        speed, steer, state = self.mission.step(now, self.scan, self.scan_stamp, self._clusters,
                                                self.yaw, self.gyro, self.travelled)
        self.gyro_bias = self.mission.gyro_bias
        self.label = state
        self.done = state == 'STOP'
        self.parked = self.mission.parking is not None and self.mission.parking.state == 'PARKED'
        return speed, steer
