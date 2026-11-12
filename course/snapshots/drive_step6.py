"""Step 6: the lane (step 4) and the corner (step 5), with the TurnGate deciding when: round and round the island."""
import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool

from robotek_common.vehicle import WHEELBASE
from robotek_course.minisim import Scene, wrap

from .arc import arc_steer, damped_steer
from .heading import GyroHeading
from .lane import GAP, lateral_bias
from .perception import perceive
from .turn_gate import ISLAND_OPEN, StraightDistance, TurnGate

CONTROL_RATE = 50.0       # Hz, on the simulation clock
SPEED = 1.28              # m/s commanded (the real car reaches 0.6 m/s with this command)
CORNER_SPEED = 1.0         # m/s commanded in a corner (0.53 m/s real)
CALIBRATE_SECONDS = 0.6   # standing still after Start, measuring the gyroscope bias


class Drive(Node):
    def __init__(self):
        super().__init__('strategy')
        self.heading = GyroHeading()
        self.gyro = 0.0
        self.samples = []
        self.time = None
        self.goal = 0.0                      # the heading to hold: each corner adds 90 degrees
        self.arc = False                     # True while the corner is in progress
        self.corners = 0                     # corners finished
        self.gate = TurnGate()               # decides when a corner starts
        self.straight = StraightDistance()   # m driven since the last corner, from the wall ahead
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
                if not self.arc:
                    self.straight.update(self.scene.front_axis)
                inner = self.scene.left                                 # the island is on the left
                if self.gate.update(self.scene, inner, self.straight.travelled, self.arc):
                    self.goal = wrap(self.goal + math.pi / 2)
                    self.arc = True
                    self.get_logger().info('Giro a la izquierda (esquina %d)' % (self.corners + 1))
                if self.arc:
                    steer, done = arc_steer(self.goal, yaw, gyro)
                    speed = CORNER_SPEED
                    if done:                                            # the corner is done
                        self.arc = False
                        self.straight.reset()
                        self.corners += 1
                else:
                    island_open = math.isfinite(inner) and inner > ISLAND_OPEN
                    keep = GAP if (not math.isfinite(inner) or island_open) else inner     # no offset at a corner
                    speed, steer = SPEED, damped_steer(self.goal + lateral_bias(keep), yaw, gyro)
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
