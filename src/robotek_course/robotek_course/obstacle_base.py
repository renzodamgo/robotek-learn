"""The base class of every Obstacle strategy in the course.

A strategy is plain Python: no ROS inside. It receives what the sensors say through four hooks and answers with
(speed, steer) every control tick. The same class runs in three places:

  * in Gazebo, inside the node ``drive.py`` (it forwards the ROS topics to these hooks);
  * in the Obstacle lab (``python3 -m robotek_evaluation.obstacle_lab --strategy my_obstacle.strategy:Strategy``);
  * in your tests (``pytest``).

You extend it and write ``update(now, dt)``. The base class takes care of the parts that are not the point of the
course: the LiDAR odometry (how far the car has driven, aligned once per scan), the scans that arrive several times
with the same time stamp, integrating the gyroscope, and waiting for Start.

What you read in ``update``:

  ``scan``, ``scan_stamp`` (s), ``new_scan``   the latest LiDAR scan (each time stamp once)
  ``gyro``                                     the latest gyroscope rate, rad/s (z, bias included)
  ``yaw``                                      the integrated gyroscope (rad), with ``gyro_bias`` taken away
  ``travelled``                                metres driven since the previous tick (0 while stopped)
  ``signs``, ``signs_stamp`` (s)               camera blocks paired with LiDAR clusters: dict(x, y, color)
                                               in the car frame (x ahead, y left), color 'red' or 'green'

and what you set: ``gyro_bias`` (after calibrating), ``label`` (the state, for telemetry and the LEDs), ``done``
(the round is over for you: stopped or parked) and ``parked``.
"""
import math

from robotek_common.lidar_odometry import LidarOdometry
from robotek_common.vehicle import DISTANCE_MODEL_ACCEL

STOPPED = 0.01          # m/s; below this (commanded and modelled) the odometry distance is ignored


class ObstacleStrategyBase:
    label = 'WAIT_START'
    done = False
    parked = False

    # In the lab the first scan is Start. In Gazebo the referee publishes /start and drive.py turns this off.
    auto_start = True

    def __init__(self):
        self.odometry = LidarOdometry(accel=DISTANCE_MODEL_ACCEL)
        self.scan = None
        self.scan_stamp = -math.inf
        self.new_scan = False
        self.gyro = 0.0
        self.gyro_bias = 0.0
        self.yaw = 0.0
        self.travelled = 0.0
        self.signs = []
        self.signs_stamp = -math.inf
        self.started = False
        self._imu_time = None
        self._last_speed = 0.0
        self._seen_first = False

    # ------------------------------------------------------------------ hooks (called by drive.py and the lab)
    def request_start(self):
        self.started = True

    def on_scan(self, scan):
        """A LiDAR scan. Returns True for the very first one (the lab uses it as Start)."""
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        if stamp > self.scan_stamp:            # the same scan can arrive more than once: align each stamp once
            self.odometry.scan(scan, self.yaw)
            self.scan_stamp, self.new_scan = stamp, True
        self.scan = scan                       # the latest copy (as the team's node does)
        first = not self._seen_first
        self._seen_first = True
        if first and self.auto_start:
            self.request_start()
        return first

    def on_imu(self, now, gyro_z):
        """A gyroscope reading (rad/s): integrated into ``yaw`` with the bias taken away."""
        if self._imu_time is not None and 0.0 < now - self._imu_time < 0.2:
            self.yaw += (gyro_z - self.gyro_bias) * (now - self._imu_time)
        self._imu_time = now
        self.gyro = gyro_z

    def on_signs(self, observations, stamp):
        """Signs seen by the camera and the LiDAR together (see ``robotek_course.obstacle_vision.associate``)."""
        self.signs, self.signs_stamp = list(observations or []), stamp

    def control(self, now, dt):
        """One control tick. Returns (speed command in m/s, steering angle in rad, + left)."""
        self.odometry.command(self._last_speed, dt)
        step = self.odometry.read()
        stopped = abs(self._last_speed) < STOPPED and abs(self.odometry.speed_model) < STOPPED
        self.travelled = 0.0 if stopped else step
        if not self.started:
            speed, steer = 0.0, 0.0
        else:
            speed, steer = self.update(now, dt)
        self._last_speed = speed
        self.new_scan = False
        return speed, steer

    # ---------------------------------------------------------------------------------- what you write
    def update(self, now, dt):
        """Decide (speed, steer) for this tick from ``self.scan``, ``self.yaw``, ``self.gyro``, ``self.travelled``
        and ``self.signs``."""
        raise NotImplementedError('write update(self, now, dt) in your Strategy')
