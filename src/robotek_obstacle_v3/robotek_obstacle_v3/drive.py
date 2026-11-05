"""The ROS node of Obstacle v3: sensors in, /cmd_vel_raw out. The decisions are in strategy.py and mission.py.

The same topics as a student's ``my_obstacle`` node, so it runs in ``obstacle_round.launch.py`` and
``car.launch.py`` with ``strategy:=robotek_obstacle_v3``. Two things differ from the course's ``drive.py``, both
taken from the v3 node of WRO-Strategies so the decisions stay the same:

* every camera frame reaches the strategy: one with no scan within ``IMAGE_PAIRING`` gives an empty list of signs
  (the course's node skips it). The strategy needs a fresh frame to keep driving;
* the 26 mission parameters of ``settings.Params`` are ROS parameters (defaults = WRO-Strategies'
  ``obstacle_v3.yaml``); ``parking`` (from the round launch) sets ``park``.

The camera is ROS parameters too, for a camera other than the one of the defaults (the USB camera measured on
8 oct 2026; ``car.launch.py strategy_params:=<file.yaml>`` sets them): ``camera_hfov``, ``camera_pitch`` and
``camera_yaw`` (rad, + = looks left of the LiDAR's zero) turn an image column into a bearing, and ``red_hsv`` /
``green_hsv`` are the colours of the signs as ``"h_min,h_max,s_min,v_min"`` (h_min > h_max wraps through 0, as red
does). ``sign_min_area`` is the smallest blob in pixels.
"""
from dataclasses import asdict
import math

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String

from robotek_common.vehicle import REAL_CAMERA_HFOV, REAL_CAMERA_PITCH, REAL_CAMERA_YAW, WHEELBASE
from robotek_course.obstacle_vision import COLOR_RANGES, associate, color_blocks

from .perception import scan_clusters
from .settings import Params
from .strategy import Strategy

CONTROL_RATE = 50.0       # Hz, on the node's clock (simulation clock in Gazebo)
IMAGE_PAIRING = 0.15      # s; a camera frame is paired with a scan at most this old


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
        self.state = self.create_publisher(
            String, '/robotek/state', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.state_sent = None
        self.pose_pub = self.create_publisher(Float32MultiArray, '/obstacle/pose', 10)   # x, y, th, progress
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image', self.on_image, qos_profile_sensor_data)
        self.create_subscription(Bool, 'start', self.on_start, 10)   # plain QoS: the referee and the car's button
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
        self.strategy.on_signs(observations, stamp)

    def on_start(self, msg):
        if msg.data and not self.strategy.started:
            self.get_logger().info('Start')
            self.strategy.request_start()

    def control(self):
        speed, steer = self.strategy.control(self.now(), 1.0 / CONTROL_RATE)
        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(speed * math.tan(steer) / WHEELBASE)
        self.cmd.publish(cmd)
        mission = self.strategy.mission
        self.pose_pub.publish(Float32MultiArray(data=[float(v) for v in (*mission.pose, mission.phi)]))
        if self.strategy.label != self.state_sent:
            self.state_sent = self.strategy.label
            self.get_logger().info('state %s' % self.state_sent)
            self.state.publish(String(data=self.state_sent))


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
