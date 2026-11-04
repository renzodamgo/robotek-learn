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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage, Image, Imu, JointState, LaserScan
from std_msgs.msg import Bool, ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from robotek_common.lidar_odometry import LidarOdometry
from robotek_common.vehicle import DISTANCE_MODEL_ACCEL, LIDAR_OFFSET_X, WHEELBASE
from ..open_mission_clock import mission_clock
from ..open_vision import OpenVision
from .mission import CONFIRMATIONS, CORNERS, LAPS, OpenMission, Params
from .recording import V5Recording
from .scene import Scene, island_wall, scan_arrays, scene_from_scan
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
        # Reliable, not best effort: a 640x480 frame is ~0.9 MB, and with the Pi's default socket buffer (208 KB) a
        # best-effort reader lost every frame once the car was busy (4 nov 2026: no direction, detector frozen).
        self.create_subscription(Image, '/camera/image', self.on_image,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(1.0 / CONTROL_RATE, self.control)
        # RViz only: the commanded steering as an arrow, and as the front wheels' angle on /joint_states when nothing
        # else publishes them (on the car; Gazebo publishes its own). Decided once, 3 s after start.
        self._steer_pub = self.create_publisher(Marker, '/open/steer', 2)
        # RViz only: how the turn is decided (front wall vs threshold, island wall and its end, the three checks).
        self._turn_pub = self.create_publisher(MarkerArray, '/open/turn', 2)
        self._joint_pub = None
        self._joint_check = self.create_timer(3.0, self._check_joint_states)

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
        overlay = self._with_progress(self.vision.overlay)
        debug = self._bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        debug.header = msg.header
        self._vision_pub.publish(debug)
        if self._vision_jpeg_pub.get_subscription_count():
            ok, jpeg = cv2.imencode('.jpg', overlay, [cv2.IMWRITE_JPEG_QUALITY, 70])
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
            if self._turn_pub.get_subscription_count():
                self._publish_turn(msg)
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
        self._publish_steer(0.0 if blind else steer)
        label = self.mission.label
        if label != self._state_sent:
            self._state_sent = label
            self._state_pub.publish(String(data=label))
        self.recording.record_mission(
            sim_seconds=self.sim_seconds, mission=self.mission, gyro=self.mission.gyro, scene=self.scene,
            vision=self.vision, odometry=self.odometry, command=cmd, travelled=self.travelled)
        self.finish_if_done()

    # ------------------------------------------------------------------ RViz
    def _with_progress(self, image):
        """The detector image with the mission's state, lap and corners in small print at the bottom."""
        done = self.mission.completed
        text = f'{self.mission.label}  vuelta {min(done // 4 + 1, LAPS)}/{LAPS}  giros {done}/{CORNERS}'
        image = image.copy()
        y = image.shape[0] - 10
        cv2.putText(image, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
        cv2.putText(image, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        return image

    def _check_joint_states(self):
        self._joint_check.cancel()
        if self.count_publishers('/joint_states') == 0:
            self._joint_pub = self.create_publisher(JointState, '/joint_states', 10)

    def _publish_steer(self, steer):
        """Arrow from the front axle (base_link) along the commanded wheel angle; front wheels turned on /joint_states."""
        stamp = self.get_clock().now().to_msg()
        if self._steer_pub.get_subscription_count():
            arrow = Marker(type=Marker.ARROW, action=Marker.ADD, ns='steer')
            arrow.header.frame_id, arrow.header.stamp = 'base_link', stamp
            arrow.pose.orientation.w = 1.0
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.02, 0.045, 0.06
            arrow.color = ColorRGBA(r=0.1, g=0.8, b=1.0, a=1.0)
            front = WHEELBASE / 2.0
            arrow.points = [Point(x=front, y=0.0, z=0.09),
                            Point(x=front + 0.28 * math.cos(steer), y=0.28 * math.sin(steer), z=0.09)]
            self._steer_pub.publish(arrow)
        if self._joint_pub is not None:
            joints = JointState(name=['front_left_steering_joint', 'front_right_steering_joint',
                                      'front_left_wheel_joint', 'front_right_wheel_joint',
                                      'rear_left_wheel_joint', 'rear_right_wheel_joint'],
                                position=[float(steer)] * 2 + [0.0] * 4)
            joints.header.stamp = stamp
            self._joint_pub.publish(joints)

    def _publish_turn(self, scan):
        """Markers (base_link) of the turn decision: front ray and threshold, island wall, its end and look-ahead."""
        mission, scene, p = self.mission, self.scene, self.mission.p
        green, red, grey = ColorRGBA(r=0.1, g=0.9, b=0.2, a=1.0), ColorRGBA(r=1.0, g=0.3, b=0.1, a=1.0), \
            ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.8)
        markers = []

        def marker(kind, ns, color, width=0.02):
            m = Marker(type=kind, action=Marker.ADD, ns=ns, id=0)
            m.header.frame_id, m.header.stamp = 'base_link', scan.header.stamp
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = width
            m.color = color
            markers.append(m)
            return m

        inner, island_open = mission.island_side(scene)
        armed, front_ok, island_ok = mission.turn_checks(scene, island_open)
        # Front wall along the track axis, and the distance at which a turn may start.
        fy = mission.forward_yaw
        ux, uy = math.cos(fy), math.sin(fy)
        if math.isfinite(scene.front_axis):
            ray = marker(Marker.LINE_LIST, 'front_ray', green if front_ok else red)
            ray.points = [Point(x=LIDAR_OFFSET_X, y=0.0, z=0.05),
                          Point(x=LIDAR_OFFSET_X + scene.front_axis * ux, y=scene.front_axis * uy, z=0.05)]
        else:
            marker(Marker.LINE_LIST, 'front_ray', grey).action = Marker.DELETE
        cx, cy, d = LIDAR_OFFSET_X + p.turn_front_distance * ux, p.turn_front_distance * uy, p.turn_front_distance
        threshold = marker(Marker.LINE_LIST, 'front_threshold', green if front_ok else grey, 0.03)
        threshold.points = [Point(x=cx - 0.45 * uy, y=cy + 0.45 * ux, z=0.05),
                            Point(x=cx + 0.45 * uy, y=cy - 0.45 * ux, z=0.05)]
        # Island wall (fitted line), its end and the look-ahead the end must be inside.
        sign = 1.0 if mission.direction == 'ccw' else -1.0
        angles, ranges, ok = scan_arrays(scan)
        wall = island_wall(angles, ranges, ok, sign) if mission.direction else None
        line = marker(Marker.LINE_LIST, 'island_wall', ColorRGBA(r=1.0, g=0.85, b=0.1, a=1.0), 0.025)
        end = marker(Marker.SPHERE, 'island_end', green if island_ok else ColorRGBA(r=1.0, g=0.85, b=0.1, a=1.0), 0.08)
        look = marker(Marker.LINE_LIST, 'lookahead', green if island_ok else grey, 0.015)
        if wall is not None and math.isfinite(wall.offset):
            far = wall.end - LIDAR_OFFSET_X if math.isfinite(wall.end) else 1.0
            line.points = [Point(x=x + LIDAR_OFFSET_X, y=sign * (wall.slope * x + wall.offset), z=0.05)
                           for x in (-0.3, far)]
            if math.isfinite(wall.end):
                end.pose.position = Point(x=wall.end, y=sign * (wall.slope * far + wall.offset), z=0.05)
            else:
                end.action = Marker.DELETE
            ahead = mission.lookahead(inner) if math.isfinite(inner) else p.corner_lookahead
            y = sign * (wall.slope * (ahead - LIDAR_OFFSET_X) + wall.offset)
            look.points = [Point(x=ahead, y=y - 0.12, z=0.05), Point(x=ahead, y=y + 0.12, z=0.05)]
        else:
            line.action = end.action = look.action = Marker.DELETE
        # The three checks, as text above the car.
        yes = lambda flag: 'SI' if flag else 'no'
        front = f'{scene.front_axis:.2f}' if math.isfinite(scene.front_axis) else '--'
        island = 'abierta' if island_open else f'fin {min(mission._hits, CONFIRMATIONS)}/{CONFIRMATIONS}'
        text = marker(Marker.TEXT_VIEW_FACING, 'turn_text', ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0), 0.07)
        text.pose.position = Point(x=0.0, y=0.0, z=0.35)
        text.text = (f'delante {yes(front_ok)} {front}/{p.turn_front_distance:.2f} m | isla {yes(island_ok)} '
                     f'({island}) | armado {yes(armed)} | giro {mission.completed}/{CORNERS} {mission.label}')
        self._turn_pub.publish(MarkerArray(markers=markers))

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
