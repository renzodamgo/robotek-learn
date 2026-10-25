"""The strategy: plain Python, no ROS. ``drive.py`` feeds it the sensors and sends its answer to the car."""
from robotek_course.base import StrategyBase
from robotek_course.minisim import Scene

from .mission import MiniMission
from .perception import perceive


class Strategy(StrategyBase):
    def __init__(self, direction='auto'):
        super().__init__()
        self.mission = MiniMission(direction)
        self.scene = Scene()

    def request_start(self):
        super().request_start()
        self.mission.request_start()

    def operator_stop(self):
        self.mission.operator_stop()

    def update(self, dt):
        if self.new_scan:
            sign = -1 if self.mission.direction == 'cw' else 1
            self.scene = perceive(self.scan, self.mission.forward_yaw, sign)
        speed, steer = self.mission.step(dt, self.scene, self.gyro, self.travelled, self.camera)
        self.yaw = self.mission.yaw
        return speed, steer

    @property
    def label(self):
        return self.mission.label

    @property
    def turns(self):
        return self.mission.completed

    @property
    def done(self):
        return self.mission.state == 'STOP'

    @property
    def stop_label(self):
        return self.mission.label if self.done else None
