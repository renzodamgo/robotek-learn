"""Step 1: drive a circle with the wheels fixed and compare it with the bicycle model."""
import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from robotek_common.vehicle import WHEELBASE

from .model import turn_radius

CONTROL_RATE = 50.0       # Hz, on the simulation clock
SPEED = 0.6               # m/s commanded (the real car reaches 0.42 m/s with this command)


class Drive(Node):
    def __init__(self):
        super().__init__('strategy')
        self.steer = self.declare_parameter('steer', 0.25).value      # rad, positive = left
        self.started = False
        self.cmd = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.create_subscription(Bool, 'start', self.on_start, 10)   # plain QoS: works with the referee and with the car's button
        self.create_timer(1.0 / CONTROL_RATE, self.control)

    def on_start(self, msg):
        if msg.data and not self.started:
            self.started = True
            self.get_logger().info('Start: con el volante a %.2f rad el modelo espera un circulo de radio %.3f m'
                                   % (self.steer, turn_radius(self.steer)))

    def control(self):
        cmd = Twist()
        if self.started:
            cmd.linear.x = SPEED
            cmd.angular.z = SPEED * math.tan(self.steer) / WHEELBASE
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
