"""The ROS node of Obstacle v3: sensors in, /cmd_vel_raw out. The decisions are in strategy.py and mission.py.

The same topics as a student's ``my_obstacle`` node, so it runs in ``obstacle_round.launch.py`` and
``car.launch.py`` with ``strategy:=robotek_obstacle_v3``. Two things differ from the course's ``drive.py``, both
taken from the v3 node of WRO-Strategies so the decisions stay the same:

* every camera frame reaches the strategy: one with no scan within ``IMAGE_PAIRING`` gives an empty list of signs
  (the course's node skips it). The strategy needs a fresh frame to keep driving;
* the 35 mission parameters of ``settings.Params`` are ROS parameters (defaults = WRO-Strategies'
  ``obstacle_v3.yaml``); ``parking`` (from the round launch) sets ``park``.

The camera is ROS parameters too, for a camera other than the one of the defaults (the USB camera measured on
8 oct 2026; ``car.launch.py strategy_params:=<file.yaml>`` sets them): ``camera_hfov``, ``camera_pitch`` and
``camera_yaw`` (rad, + = looks left of the LiDAR's zero) turn an image column into a bearing, and ``red_hsv`` /
``green_hsv`` are the colours of the signs as ``"h_min,h_max,s_min,v_min"`` (h_min > h_max wraps through 0, as red
does). ``sign_min_area`` is the smallest blob in pixels.

For RViz, only while someone watches them: ``/obstacle/debug_image/compressed`` (the frame with the red and green
blobs, thick when paired with a LiDAR cluster, the state and the steering) and ``/obstacle/signs`` (the paired signs
as cylinders and the steering as an arrow, in base_link).
"""
from dataclasses import asdict
import math

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, Imu, LaserScan
from std_msgs.msg import Bool, Float32MultiArray, Int32, String
from visualization_msgs.msg import Marker, MarkerArray

from robotek_common.vehicle import REAL_CAMERA_HFOV, REAL_CAMERA_PITCH, REAL_CAMERA_YAW, WHEELBASE
from robotek_course.obstacle_vision import COLOR_RANGES, associate, color_blocks

from .lot_exit import DEFAULT_STEPS, parse_steps
from .perception import scan_clusters
from .settings import Params
from .strategy import Strategy

CONTROL_RATE = 50.0       # Hz, on the node's clock (simulation clock in Gazebo)
IMAGE_PAIRING = 0.15      # s; a camera frame is paired with a scan at most this old
DEBUG_PERIOD = 0.2        # s; the detections image at most 5 times a second (JPEG at 25 fps made the camera stale, 8 nov)


def hsv_bands(spec):
    """``"h_min,h_max,s_min,v_min"`` -> the [(low, high)] bands of ``color_blocks``; h_min > h_max wraps through 0."""
    h_min, h_max, s_min, v_min = (int(float(v)) for v in spec.split(','))
    if h_min <= h_max:
        return [((h_min, s_min, v_min), (h_max, 255, 255))]
    return [((0, s_min, v_min), (h_max, 255, 255)), ((h_min, s_min, v_min), (179, 255, 255))]


def hsv_spec(bands):
    """The inverse of ``hsv_bands`` for the default ranges (used as the parameters' defaults)."""
    (low, high), *rest = bands
    if rest:                                   # wrapped: (0..h_max) and (h_min..179)
        return '%d,%d,%d,%d' % (rest[-1][0][0], high[0], low[1], low[2])
    return '%d,%d,%d,%d' % (low[0], high[0], low[1], low[2])


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Drive(Node):
    def __init__(self):
        super().__init__('strategy')
        parking = self.declare_parameter('parking', False).value
        defaults = asdict(Params())
        values = {name: float(self.declare_parameter(name, float(value)).value) for name, value in defaults.items()}
        # Only the parameters changed by hand: a default park=0 must not cancel parking:=true.
        params = {name: value for name, value in values.items() if value != defaults[name]}
        self.strategy = Strategy(self.declare_parameter('direction', 'auto').value, parking=parking, params=params)
        steps = str(self.declare_parameter('lot_exit_steps', DEFAULT_STEPS).value)
        parse_steps(steps)                               # a bad sequence stops the node at launch, not in the lot
        self.strategy.mission.lot_exit_steps = steps
        self.strategy.auto_start = False                 # the referee (or the button on the car) publishes /start
        self.camera = dict(
            hfov=float(self.declare_parameter('camera_hfov', REAL_CAMERA_HFOV).value),
            pitch=float(self.declare_parameter('camera_pitch', REAL_CAMERA_PITCH).value),
            yaw=float(self.declare_parameter('camera_yaw', REAL_CAMERA_YAW).value),
            ranges={color: hsv_bands(self.declare_parameter(color + '_hsv', hsv_spec(COLOR_RANGES[color])).value)
                    for color in ('red', 'green')},
            min_area=int(self.declare_parameter('sign_min_area', 30).value))
        self.bridge = CvBridge()
        self.scan_msg = None
        self.cmd = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.estop = self.create_publisher(Bool, '/robotek/estop', 10)   # the lot exit's hard brake
        self.lot_logged = 0          # lot exit log entries already printed
        self.events_logged = 0       # recover log entries already printed
        self.park_state = None     # last parking sub-state logged
        self.state = self.create_publisher(
            String, '/robotek/state', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.state_sent = None
        self.pose_pub = self.create_publisher(Float32MultiArray, '/obstacle/pose', 10)   # x, y, th, progress
        self.debug_pub = self.create_publisher(CompressedImage, '/obstacle/debug_image/compressed', 1)
        self.signs_pub = self.create_publisher(MarkerArray, '/obstacle/signs', 1)
        self.steer = 0.0
        self.debug_sent = -1e9
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image', self.on_image, qos_profile_sensor_data)
        self.create_subscription(Bool, 'start', self.on_start, 10)   # plain QoS: the referee and the car's button
        # The push button on the Pi's GPIO 16 (gpio_button node, True on each press) also starts the round (9 nov 2026).
        self.create_subscription(Bool, '/robotek/gpio_button', self.on_start, 10)
        # Rear VL53L0X (rear_range node, mm, -1 = nothing in range): the lot exit backs up on it (start_lot).
        self.create_subscription(Int32, '/robotek/rear_range_mm', self.on_rear, 10)
        self.create_timer(1.0 / CONTROL_RATE, self.control)
        p = self.strategy.mission.p
        self.get_logger().info('Obstacle v3 ready (park=%g, steer_limit=%.3f rad, changed: %s; camera %s)'
                               % (p.park, p.steer_limit, params, self.camera))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_scan(self, msg):
        self.scan_msg = msg
        self.strategy.on_scan(msg)

    def on_imu(self, msg):
        self.strategy.on_imu(stamp_of(msg), msg.angular_velocity.z)

    def on_image(self, msg):
        """Red and green blocks of the frame paired with the LiDAR clusters at the same bearing."""
        stamp = stamp_of(msg)
        observations = []
        if self.scan_msg is not None and abs(stamp - stamp_of(self.scan_msg)) <= IMAGE_PAIRING:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            blocks = color_blocks(bgr, **self.camera)
            observations = associate(blocks, scan_clusters(self.scan_msg))
            if self.debug_pub.get_subscription_count() and stamp - self.debug_sent >= DEBUG_PERIOD:
                self.debug_sent = stamp
                self.publish_debug(msg, bgr, blocks, observations)
        self.strategy.on_signs(observations, stamp)
        if self.signs_pub.get_subscription_count():
            self.publish_signs(msg, observations)

    def publish_debug(self, msg, bgr, blocks, observations):
        """The frame with every red and green blob (thick = paired with a LiDAR cluster), the state and the steering."""
        paired = {o['box'] for o in observations}
        for block in blocks:
            x, y, w, h = block['box']
            color = (0, 200, 0) if block['color'] == 'green' else (0, 0, 255)
            thick = 3 if block['box'] in paired else 1
            cv2.rectangle(bgr, (x, y), (x + w, y + h), color, thick)
            cv2.putText(bgr, '%s %+.0f' % (block['color'][0].upper(), math.degrees(block['bearing'])), (x, max(12, y - 3)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        for o in observations:
            cv2.putText(bgr, '%s %.2f,%+.2f' % (o['color'][0].upper(), o['x'], o['y']), (8, bgr.shape[0] - 10 - 16 *
                        observations.index(o)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
        cv2.putText(bgr, '%s  signs %d  steer %+.0f' % (self.strategy.label, len(observations), math.degrees(self.steer)),
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
        out = CompressedImage(format='jpeg', data=cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes())
        out.header = msg.header
        self.debug_pub.publish(out)

    def publish_signs(self, msg, observations):
        """The paired signs as cylinders and the commanded steering as an arrow, in base_link."""
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for i, o in enumerate(observations):
            m = Marker(ns='signs', id=i, type=Marker.CYLINDER, action=Marker.ADD)
            m.header.frame_id, m.header.stamp = 'base_link', msg.header.stamp
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(o['x']), float(o['y']), 0.05
            m.scale.x = m.scale.y = 0.05
            m.scale.z = 0.1
            m.color.a, m.color.r, m.color.g = 0.9, float(o['color'] == 'red'), float(o['color'] == 'green')
            markers.markers.append(m)
        arrow = Marker(ns='steer', id=0, type=Marker.ARROW, action=Marker.ADD)
        arrow.header.frame_id, arrow.header.stamp = 'base_link', msg.header.stamp
        arrow.pose.position.x = float(WHEELBASE)
        arrow.pose.orientation.z, arrow.pose.orientation.w = math.sin(self.steer / 2), math.cos(self.steer / 2)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.25, 0.02, 0.02
        arrow.color.a, arrow.color.r, arrow.color.g, arrow.color.b = 1.0, 1.0, 1.0, 0.0
        markers.markers.append(arrow)
        self.signs_pub.publish(markers)

    def on_rear(self, msg):
        self.strategy.mission.rear(self.now(), int(msg.data))

    def on_start(self, msg):
        if msg.data and not self.strategy.started:
            self.get_logger().info('Start')
            self.strategy.request_start()

    def control(self):
        speed, steer = self.strategy.control(self.now(), 1.0 / CONTROL_RATE)
        self.steer = steer
        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(speed * math.tan(steer) / WHEELBASE)
        self.cmd.publish(cmd)
        mission = self.strategy.mission
        self.pose_pub.publish(Float32MultiArray(data=[float(v) for v in (*mission.pose, mission.phi)]))
        self._log_parking(mission)
        if mission.brake:                                # the front recover's hard brake
            self.estop.publish(Bool(data=True))
        for entry in mission.events[self.events_logged:]:
            self.get_logger().info('recover: %s' % entry)
        self.events_logged = len(mission.events)
        if mission.lot_exit is not None:
            if mission.lot_exit.brake:
                self.estop.publish(Bool(data=True))
            for entry in mission.lot_exit.log[self.lot_logged:]:
                self.get_logger().info('lot exit: %s' % entry)
            self.lot_logged = len(mission.lot_exit.log)
        if self.strategy.label != self.state_sent:
            self.state_sent = self.strategy.label
            if self.state_sent == 'WAIT_SENSOR':
                ages = self.strategy.stale(self.now())
                self.get_logger().warn('state WAIT_SENSOR: stale %s' % ', '.join(
                    '%s %.2f s' % (name, age) for name, age in sorted(ages.items())))
            else:
                self.get_logger().info('state %s' % self.state_sent)
            self.state.publish(String(data=self.state_sent))


    def _log_parking(self, mission):
        """The parking's own steps and events: each sub-state (PARK_APPROACH, PARK_MOVE, ...) with the pose, the
        log of park.py / park_fixed.py (start pose, LiDAR correction, end) and the verdict, PARKED or PARK_FAIL and
        why (on the car on 8 nov 2026 a DONE could be either)."""
        parking = mission.parking
        if parking is None:
            return
        # The fixed park's pose against the lot (with its LiDAR correction), else the mission's.
        own = parking._corrected(*parking.raw) if getattr(parking, 'raw', None) else mission.pose
        pose = ' '.join('%.3f' % v for v in own)
        if parking.state != self.park_state:
            self.park_state = parking.state
            move = getattr(parking, 'current', None)
            what = '' if move is None else ' move steer %.3f dir %+.0f target %s' % (move[0], move[1], move[3][0])
            self.get_logger().info('park %s at %s%s' % (parking.state, pose, what))
        for event in parking.log:
            self.get_logger().info('park event %s' % {k: ([round(float(x), 4) for x in v] if isinstance(v, (list, tuple))
                                                          else v) for k, v in event.items()})
            if event.get('event') == 'park_end':
                inside = parking.lot.parked(*own, inside=0.0, parallel=math.radians(90.0))
                why = 'parked: inside and parallel (own pose)' if event.get('parked') else (
                    'not parallel: heading %.1f deg' % math.degrees(own[2]) if inside
                    else 'not inside the lot (own pose)')
                self.get_logger().info('park verdict %s: %s, %d moves' % (
                    'PARKED' if event.get('parked') else 'PARK_FAIL', why, event.get('moves', 0)))
        parking.log.clear()


def main(args=None):
    rclpy.init(args=args)
    node = Drive()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        if rclpy.ok():
            node.cmd.publish(Twist())
            rclpy.shutdown()
