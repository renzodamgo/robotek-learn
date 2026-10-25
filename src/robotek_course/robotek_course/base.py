"""The base class of every strategy in the course.

A strategy is plain Python: no ROS inside. It receives what the sensors say through four hooks and answers with
(speed, steer) every control tick. The same class runs in three places, which is why it is written this way:

  * in Gazebo, inside the node ``drive.py`` (it forwards the ROS topics to these hooks);
  * in the Open lab (``python3 -m robotek_evaluation.open_lab --strategy my_open.strategy:Strategy``);
  * in your tests (``pytest``).

You extend it and write ``update``. The base class takes care of the parts that are not the point of the course:
the LiDAR odometry (how far the car has driven), the scans that arrive several times with the same time stamp, and
waiting for Start.
"""
from robotek_common.lidar_odometry import LidarOdometry
from robotek_common.vehicle import DISTANCE_MODEL_ACCEL


class StrategyBase:
    # What the rest of the system reads (telemetry, the lab's report, LEDs).
    label = 'WAIT_START'
    turns = 0
    done = False
    stop_label = None

    # In the lab the first scan is Start. In Gazebo the round recorder publishes /start and drive.py turns this off.
    auto_start = True

    def __init__(self):
        self.odometry = LidarOdometry(accel=DISTANCE_MODEL_ACCEL)
        self.scan = None            # the latest scan (each time stamp once)
        self.new_scan = False       # True for the first tick after a scan arrived
        self.gyro = 0.0             # the latest gyroscope reading, rad/s (z), bias included
        self.camera = 0             # +1 blue lines (counter-clockwise), -1 orange (clockwise), 0 unknown
        self.travelled = 0.0        # metres driven since the previous tick, from the LiDAR odometry
        self.started = False
        self.yaw = 0.0              # YOUR heading estimate (rad): set it from step 2 on; the odometry needs it
        self._stamp = None
        self._seen_first = False

    # ------------------------------------------------------------------ hooks (called by drive.py and the lab)
    def request_start(self):
        self.started = True

    def on_scan(self, scan):
        """A LiDAR scan. Returns True for the very first one (the lab uses it as Start)."""
        self.odometry.scan(scan, self.yaw)
        stamp = (scan.header.stamp.sec, scan.header.stamp.nanosec)
        if stamp != self._stamp:             # the same scan can arrive more than once: use each time stamp once
            self._stamp = stamp
            self.scan, self.new_scan = scan, True
        first = not self._seen_first
        self._seen_first = True
        if first and self.auto_start:
            self.request_start()
        return first

    def on_imu(self, now, gyro_z):
        self.gyro = gyro_z

    def on_camera(self, direction):
        self.camera = direction

    def control(self, dt, camera_age=0.0):
        """One control tick. Returns (speed command in m/s, steering angle in rad)."""
        self.travelled = self.odometry.read()
        if camera_age > 0.5:
            self.camera = 0                  # an old vote is no vote
        speed, steer = self.update(dt)
        self.odometry.command(speed, dt)
        self.new_scan = False
        return speed, steer

    # ---------------------------------------------------------------------------------- what you write
    def update(self, dt):
        """Decide (speed, steer) for this tick from ``self.scan``, ``self.gyro``, ``self.travelled``, ``self.camera``."""
        raise NotImplementedError('write update(self, dt) in your Strategy')
