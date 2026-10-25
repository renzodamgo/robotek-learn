"""Step 5: along the island wall (step 4), then ONE 90 degree corner to the left, and stop at the next wall."""
import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool

from robotek_common.vehicle import WHEELBASE
from robotek_course.minisim import STEER_LIMIT, Scene, real_speed, wrap

from .arc import arc_steer
from .heading import GyroHeading
from .lane import GAP, lane_steer
from .perception import perceive

CONTROL_RATE = 50.0       # Hz, on the simulation clock
SPEED = 1.28              # m/s commanded (the real car reaches 0.6 m/s with this command)
CALIBRATE_SECONDS = 0.6   # standing still after Start, measuring the gyroscope bias
TURN_FRONT = 1.19         # m: the corner starts when the wall ahead is this near
TURN_TOLERANCE = 0.12     # rad: the corner is done within this of the goal heading


class Drive(Node):
    def __init__(self):
        super().__init__('strategy')
        self.heading = GyroHeading()
        self.gyro = 0.0
        self.samples = []
        self.time = None
        self.stopped = False
        self.goal = self.ref = 0.0           # the heading to hold, and the ramped reference of the corner
        self.arc = False                     # True while the corner is in progress
        self.corners = 0                     # corners finished
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
            self.scene = perceive(msg, forward_yaw=wrap(self.goal - self.heading.yaw), sign=1)

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
                gyro = self.gyro - self.heading.bias
                if not self.arc and self.scene.front_axis <= TURN_FRONT:
                    if self.corners == 0:
                        self.goal = wrap(self.goal + math.pi / 2)        # a left corner: the goal turns 90 degrees
                        self.arc = True
                        self.get_logger().info('Giro a la izquierda')
                    else:
                        self.stopped = True
                if self.arc:
                    rate = real_speed(SPEED) * math.tan(STEER_LIMIT) / WHEELBASE      # yaw rate at full lock
                    steer, self.ref = arc_steer(self.goal, self.ref, yaw, gyro, rate, dt, sign=1)
                    speed = SPEED
                    if abs(wrap(self.goal - yaw)) <= TURN_TOLERANCE:                  # the corner is done
                        self.arc, self.ref = False, self.goal
                        self.corners += 1
                elif not self.stopped:
                    inner = self.scene.left if math.isfinite(self.scene.left) else GAP
                    speed, steer = SPEED, lane_steer(self.goal, yaw, inner, sign=1)
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
