"""The ROS node: sensors in, /cmd_vel_raw out. The decisions are in strategy.py."""
import math

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Bool, String

from robotek_common.vehicle import WHEELBASE
from robotek_course.watchdog import SensorWatchdog
from robotek_course.vision import line_lengths

from .direction import DirectionVote
from .strategy import Strategy

CONTROL_RATE = 50.0       # Hz, on the simulation clock


class Drive(Node):
    def __init__(self):
        super().__init__('strategy')
        self.strategy = Strategy(self.declare_parameter('direction', 'auto').value)
        self.watchdog = SensorWatchdog()                   # brake if the LiDAR or the gyroscope go quiet
        self.strategy.auto_start = False                 # the referee (or the button on the car) publishes /start
        self.vote = DirectionVote()
        self.bridge = CvBridge()
        self.image_time = None
        self.image_stamp = None
        self.cmd = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.state = self.create_publisher(
            String, '/robotek/state', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.state_sent = None
        self.create_subscription(Image, '/camera/image', self.on_image, qos_profile_sensor_data)
        self.create_subscription(Bool, '/robotek/stop', self.on_stop, 10)      # the button again: stop the car
        self.create_subscription(Bool, 'start', self.on_start, 10)   # plain QoS: works with the referee and with the car's button
        self.create_timer(1.0 / CONTROL_RATE, self.control)

    def on_scan(self, msg):
        self.watchdog.scan(self.now())
        self.strategy.on_scan(msg)

    def on_imu(self, msg):
        self.watchdog.imu(self.now())
        self.strategy.on_imu(self.now(), msg.angular_velocity.z)

    def on_image(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.image_stamp is not None and stamp - self.image_stamp < 0.09:
            return                                       # one frame in two is enough
        self.image_stamp, self.image_time = stamp, self.now()
        blue, orange = line_lengths(self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
        self.strategy.on_camera(self.vote.update(blue, orange))

    def on_start(self, msg):
        if msg.data and not self.strategy.started:
            self.get_logger().info('Start')
            self.strategy.request_start()

    def on_stop(self, msg):
        if msg.data:
            self.get_logger().info('Operator stop')
            self.strategy.operator_stop()

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def control(self):
        if self.watchdog.stale(self.now()):
            self.get_logger().warning('A sensor went quiet: braking until it is back', throttle_duration_sec=2.0)
            speed, steer = 0.0, 0.0
        else:
            age = self.now() - self.image_time if self.image_time is not None else math.inf
            speed, steer = self.strategy.control(1.0 / CONTROL_RATE, age)
        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(speed * math.tan(steer) / WHEELBASE)
        self.cmd.publish(cmd)
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
