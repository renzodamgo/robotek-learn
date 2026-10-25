"""Steering angle (std_msgs/Float32, rad, +left) -> RRC Lite servo only, for bench tests.

Same mapping and +-25 deg limit as base_adapter, but it never commands the motor.
Centres the wheels when the angle stops arriving (watchdog) and on shutdown.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from .mapping import ServoCalibration, servo_us

try:
    from rrc_lite_msgs.msg import SetAckerServoState
except ImportError:  # only installed on the robot (HiWonder image)
    SetAckerServoState = None


class ServoSteer(Node):
    def __init__(self):
        super().__init__('servo_steer')
        p = {k: self.declare_parameter(k, v).value for k, v in dict(
            servo_topic='/rrc/acker_servo/set_state', servo_duration=.05,
            watchdog=.5, servo_center_us=1500., servo_gain_pos=550./29., servo_gain_neg=500./29.,
            servo_sign=-1., servo_min_us=1000., servo_max_us=2050., steer_limit_deg=25.).items()}
        self.p = p
        self.servo = ServoCalibration(p['servo_center_us'], p['servo_gain_pos'], p['servo_gain_neg'],
                                      p['servo_sign'], p['servo_min_us'], p['servo_max_us'],
                                      p['steer_limit_deg'])
        if SetAckerServoState is None:
            raise RuntimeError('rrc_lite_msgs missing: cannot command the servo')
        self.pub = self.create_publisher(SetAckerServoState, p['servo_topic'], 10)
        self.create_subscription(Float32, 'steer', self.on_steer, 10)
        self.last_us = None
        self.last = self.get_clock().now()
        self.create_timer(.1, self.on_watchdog)
        self.send(0.)

    def on_steer(self, msg):
        self.last = self.get_clock().now()
        self.send(msg.data)

    def send(self, steer_rad):
        us = int(round(servo_us(steer_rad, self.servo)))
        if us == self.last_us:
            return
        command = SetAckerServoState()
        command.position = us
        command.duration = float(self.p['servo_duration'])
        self.pub.publish(command)
        self.last_us = us

    def on_watchdog(self):
        if (self.get_clock().now()-self.last).nanoseconds*1e-9 > self.p['watchdog']:
            self.send(0.)


def main(args=None):
    rclpy.init(args=args)
    node = ServoSteer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.last_us = None
        node.send(0.)
        node.destroy_node()
        rclpy.try_shutdown()
