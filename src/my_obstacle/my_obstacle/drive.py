"""The ROS node: sensors in, /cmd_vel_raw out. The decisions are in strategy.py."""
import math

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String

from robotek_common.vehicle import REAL_CAMERA_HFOV, REAL_CAMERA_PITCH, REAL_CAMERA_YAW, WHEELBASE
from robotek_course.obstacle_vision import associate, color_blocks, scan_clusters
from robotek_course.watchdog import SensorWatchdog

from .strategy import Strategy

CONTROL_RATE = 50.0       # Hz, on the simulation clock
IMAGE_PAIRING = 0.15      # s; a camera frame is paired with a scan at most this old


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Drive(Node):
    def __init__(self):
        super().__init__('strategy')
        parking = self.declare_parameter('parking', False).value
        self.strategy = Strategy(self.declare_parameter('direction', 'auto').value, parking=parking)
        self.strategy.auto_start = False                 # the referee (or the button on the car) publishes /start
        self.watchdog = SensorWatchdog()                   # brake if the LiDAR or the gyroscope go quiet
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

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_scan(self, msg):
        self.watchdog.scan(self.now())
        self.scan_msg = msg
        self.strategy.on_scan(msg)

    def on_imu(self, msg):
        self.watchdog.imu(self.now())
        self.strategy.on_imu(stamp_of(msg), msg.angular_velocity.z)

    def on_image(self, msg):
        """Red and green blocks of the frame paired with the LiDAR clusters at the same bearing."""
        stamp = stamp_of(msg)
        if self.scan_msg is None or abs(stamp - stamp_of(self.scan_msg)) > IMAGE_PAIRING:
            return
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        blocks = color_blocks(bgr, hfov=REAL_CAMERA_HFOV, pitch=REAL_CAMERA_PITCH, yaw=REAL_CAMERA_YAW)
        self.strategy.on_signs(associate(blocks, scan_clusters(self.scan_msg)), stamp)

    def on_start(self, msg):
        if msg.data and not self.strategy.started:
            self.get_logger().info('Start')
            self.strategy.request_start()

    def control(self):
        if self.watchdog.stale(self.now()):
            self.get_logger().warning('A sensor went quiet: braking until it is back', throttle_duration_sec=2.0)
            speed, steer = 0.0, 0.0
        else:
            speed, steer = self.strategy.control(self.now(), 1.0 / CONTROL_RATE)
        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(speed * math.tan(steer) / WHEELBASE)
        self.cmd.publish(cmd)
        mission = self.strategy.mission
        self.pose_pub.publish(Float32MultiArray(data=[float(v) for v in (*mission.pose, mission.phi)]))
        if self.strategy.label != self.state_sent:
            self.state_sent = self.strategy.label
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
