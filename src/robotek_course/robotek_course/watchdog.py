"""Brake when a sensor goes quiet.

A strategy decides from the latest scan and gyroscope reading. If either stops arriving (a loose cable, a crashed
driver) the car would keep driving on old data. ``SensorWatchdog`` says when that has happened so the node can send
speed 0 until the data are back. The limits are the ones the team used on the real car.
"""


class SensorWatchdog:
    def __init__(self, scan_max_age=0.35, imu_max_age=0.2):
        self.scan_max_age, self.imu_max_age = scan_max_age, imu_max_age
        self.scan_time = self.imu_time = None

    def scan(self, now):
        self.scan_time = now

    def imu(self, now):
        self.imu_time = now

    def stale(self, now):
        """True when a sensor has never published or its last message is older than its limit (seconds)."""
        return any(last is None or now - last > limit
                   for last, limit in ((self.scan_time, self.scan_max_age), (self.imu_time, self.imu_max_age)))
