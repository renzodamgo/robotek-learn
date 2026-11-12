"""Step 4: drive along the island wall at a fixed distance, and stop when the wall ahead is near."""
import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool

from robotek_common.vehicle import WHEELBASE
from robotek_course.minisim import Scene, wrap

from .heading import GyroHeading
from .lane import GAP, lane_steer
from .perception import perceive

CONTROL_RATE = 50.0       # Hz, on the simulation clock
SPEED = 1.28              # m/s commanded (the real car reaches 0.6 m/s with this command)
CALIBRATE_SECONDS = 0.6   # standing still after Start, measuring the gyroscope bias
TURN_FRONT = 1.19         # m: where a corner will start (step 6); for now the car stops here


class Drive(Node):
    def __init__(self):
        super().__init__('strategy')
        self.heading = GyroHeading()
        self.gyro = 0.0
        self.samples = []
        self.time = None
        self.stopped = False
        self.scene = Scene()
        self.stamp = None
        self.cmd = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Bool, 'start', self.on_start, 10)   # plain QoS: works with the referee and with the car's button
        self.create_timer(1.0 / CONTROL_RATE, self.control)

    def on_scan(self, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp != self.stamp:              # each scan once
            self.stamp = stamp
            self.scene = perceive(msg, forward_yaw=wrap(0.0 - self.heading.yaw), sign=1)

    def on_imu(self, msg):
        self.gyro = msg.angular_velocity.z

    def on_start(self, msg):
        if msg.data and self.time is None:
            self.get_logger().info('Start')
            self.time = 0.0

    def control(self):
        dt = 1.0 / CONTROL_RATE
        speed, steer = 0.0, 0.0
        if self.time is not None:
            self.time += dt
            if self.time < CALIBRATE_SECONDS:
                self.samples.append(self.gyro)
            else:
                if self.samples:
                    self.heading.calibrate(self.samples)
                    self.samples = []
                yaw = self.heading.update(self.gyro, dt)
                self.stopped = self.stopped or self.scene.front_axis <= TURN_FRONT
                if not self.stopped:
                    inner = self.scene.left if math.isfinite(self.scene.left) else GAP     # the island is on the left
                    speed, steer = SPEED, lane_steer(0.0, yaw, inner, sign=1)
        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(speed * math.tan(steer) / WHEELBASE)
        self.cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Drive()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        if rclpy.ok():
            node.cmd.publish(Twist())       # leave the car stopped
        rclpy.try_shutdown()
