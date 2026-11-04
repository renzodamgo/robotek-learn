#!/usr/bin/env python3
"""ROS wiring of the Open v5 mission: LiDAR, gyroscope, camera and Start -> /cmd_vel_raw.

On the car, car.launch.py starts it as the strategy (it remaps /scan, /imu and /camera/image to the car's topics):

    ros2 launch robotek_course car.launch.py strategy:=robotek_open executable:=open_v5

and base_adapter turns /cmd_vel_raw into the servo pulse (measured steering table) and the motor speed. One Scene is
built per distinct LiDAR stamp (the Gazebo runs deliver every scan three times).
"""
from dataclasses import fields
import math
import os
import signal
import time

import cv2
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CompressedImage, Image, Imu, LaserScan
from std_msgs.msg import Bool, String

from robotek_common.lidar_odometry import LidarOdometry
from robotek_common.vehicle import DISTANCE_MODEL_ACCEL, WHEELBASE
from ..open_mission_clock import mission_clock
from ..open_vision import OpenVision
from .mission import OpenMission, Params
from .recording import V5Recording
from .scene import Scene, scene_from_scan
from .settings import PARAMETERS

CONTROL_RATE = 50.0


class OpenV5Node(Node):
    def __init__(self):
        super().__init__('open_strategy')
        for name, default in PARAMETERS.items():
            self.declare_parameter(name, default)
        self.cfg = {name: self.get_parameter(name).value for name in PARAMETERS}
        self.mission = OpenMission(Params(**{f.name: self.cfg[f.name] for f in fields(Params)}))
        cfg = self.cfg
        self.vision = OpenVision(
            40.0, 0.25, cfg['direction_confirmations'],
            blue_hsv=(cfg['vision_blue_h_min'], cfg['vision_blue_h_max'],
                      cfg['vision_blue_s_min'], cfg['vision_blue_v_min']),
            orange_hsv=(cfg['vision_orange_h_min'], cfg['vision_orange_h_max'],
                        cfg['vision_orange_s_min'], cfg['vision_orange_v_min']),
            floor_mask=cfg['vision_floor_mask'], wall_v_max=cfg['vision_wall_v_max'],
            wall_min_rows=cfg['vision_wall_min_rows'])
        from cv_bridge import CvBridge
        self._bridge = CvBridge()
        self.odometry = LidarOdometry(accel=DISTANCE_MODEL_ACCEL)
        telemetry = cfg['telemetry_path']
        if telemetry == 'auto':
            telemetry = os.path.join(os.path.expanduser('~/runs'), time.strftime('%Y%m%d-%H%M%S') + '-open-v5',
                                     'telemetry.csv')
        self.recording = V5Recording(telemetry, cfg['telemetry_sync_seconds'])
        self.scene, self._scene_stamp = Scene(), None
        self._gyro_z = 0.0
        self._scan_time = self._imu_time = self._image_time = self._image_stamp = None
        self._clock_received, self.sim_seconds, self.travelled = False, 0.0, 0.0
        self._done_mark, self._done_clock, self._state_sent = None, 'wall', None
        self.get_logger().info(
            f'Open v5 | direction={self.mission.p.direction} cruise={self.mission.p.cruise_speed} m/s'
            f' | telemetry {telemetry or "off"}')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)   # base_adapter's input (car.launch.py)
        self._vision_pub = self.create_publisher(Image, '/camera/direction_debug', 2)
        # The same image as JPEG, for RViz over Wi-Fi (the raw one is ~0.9 MB per frame); encoded only while watched.
        self._vision_jpeg_pub = self.create_publisher(CompressedImage, '/camera/direction_debug/compressed', 2)
        self._state_pub = self.create_publisher(
            String, '/robotek/state', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Bool, 'start', self.on_start, 10)
        self.create_subscription(Bool, '/robotek/stop', self.on_operator_stop, 10)
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.create_subscription(Image, '/camera/image', self.on_image, qos_profile_sensor_data)
        self.create_timer(1.0 / CONTROL_RATE, self.control)

    # ------------------------------------------------------------- callbacks
    def on_image(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._image_stamp is not None and stamp - self._image_stamp < 0.09:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().warning(f'Camera conversion failed: {error}', throttle_duration_sec=2.0)
            return
        self._image_stamp = stamp
        self._image_time = self.get_clock().now().nanoseconds * 1e-9
        self.vision.update(frame, stamp)
        debug = self._bridge.cv2_to_imgmsg(self.vision.overlay, encoding='bgr8')
        debug.header = msg.header
        self._vision_pub.publish(debug)
        if self._vision_jpeg_pub.get_subscription_count():
            ok, jpeg = cv2.imencode('.jpg', self.vision.overlay, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                self._vision_jpeg_pub.publish(CompressedImage(header=msg.header, format='jpeg', data=jpeg.tobytes()))
        self.recording.record_camera(stamp, self.vision)

    def on_scan(self, msg):
        self._scan_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odometry.scan(msg, self.mission.yaw)
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp != self._scene_stamp:
            self._scene_stamp = stamp
            self.scene = scene_from_scan(msg, self.mission.forward_yaw, self.mission.direction or 'ccw')
        self.recording.record_scan(msg, self.sim_seconds)
        if self.cfg['auto_start'] and not self.mission.started:
            self.get_logger().info('scan received -> requesting start')
            self.request_start('auto_first_scan')

    def on_imu(self, msg):
        self._imu_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._gyro_z = msg.angular_velocity.z

    def on_clock(self, msg):
        self.sim_seconds = msg.clock.sec + msg.clock.nanosec * 1e-9
        self._clock_received = True

    def request_start(self, source):
        if not self.mission.started:
            self.recording.record_event('start', source, self.sim_seconds)
            self.mission.request_start()

    def on_start(self, msg):
        if msg.data:
            self.request_start('start_topic')

    def on_operator_stop(self, msg):
        if msg.data and self.mission.label != 'OPERATOR_STOP':
            self.recording.record_event('operator_stop', 'stop_topic', self.sim_seconds)
            self.mission.operator_stop()

    def sensors_fresh(self, now):
        for limit, stamp in ((self.cfg['scan_max_age'], self._scan_time),
                             (self.cfg['imu_max_age'], self._imu_time)):
            if limit > 0.0 and (stamp is None or now - stamp > limit):
                return False
        return True

    # --------------------------------------------------------------- control
    def control(self):
        dt = 1.0 / CONTROL_RATE
        now = self.get_clock().now().nanoseconds * 1e-9
        camera = 0
        if self._image_time is not None and now - self._image_time <= 0.5:
            camera = self.vision.direction
        distance = self.odometry.read()
        self.travelled += distance
        self.odometry.read_measured()
        if self.sensors_fresh(now):
            speed, steer = self.mission.step(dt, self.scene, self._gyro_z, distance, camera)
        else:
            speed, steer = 0.0, 0.0     # stale LiDAR/IMU: brake until they recover
            self.get_logger().warning('Stale LiDAR/IMU: braking until sensors recover.',
                                      throttle_duration_sec=1.0)
        self.odometry.command(speed, dt)
        cmd = Twist()
        blind = self.scene.left == math.inf and self.scene.right == math.inf
        cmd.linear.x = 0.0 if blind else float(speed)
        cmd.angular.z = 0.0 if blind else float(speed * math.tan(steer) / WHEELBASE)
        self.cmd_pub.publish(cmd)
        label = self.mission.label
        if label != self._state_sent:
            self._state_sent = label
            self._state_pub.publish(String(data=label))
        self.recording.record_mission(
            sim_seconds=self.sim_seconds, mission=self.mission, gyro=self.mission.gyro, scene=self.scene,
            vision=self.vision, odometry=self.odometry, command=cmd, travelled=self.travelled)
        self.finish_if_done()

    def finish_if_done(self):
        if self.mission.label == 'DONE':
            if self._done_mark is None:
                self._done_clock, self._done_mark = mission_clock(
                    self._clock_received, self.sim_seconds, time.monotonic())
            now = self.sim_seconds if self._done_clock == 'sim' else time.monotonic()
            if self.cfg['shutdown_on_done'] and now - self._done_mark >= self.cfg['done_hold']:
                self.get_logger().info(f'mission complete -> shutting down after {self.cfg["done_hold"]:.1f} s')
                rclpy.shutdown()
        elif self.mission.label == 'DIRECTION_FAULT':
            self.get_logger().error('Direction unresolved; stopped without guessing.')
            rclpy.shutdown()


def main(args=None):
    # Defer external shutdown until the current callback returns (a SIGINT inside gzip.write corrupts the scans).
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = OpenV5Node()
    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    previous = {s: signal.signal(s, request_stop) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.recording.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for s, handler in previous.items():
            signal.signal(s, handler)


if __name__ == '__main__':
    main()
