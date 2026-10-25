"""/cmd_vel (Twist) -> RRC Lite servo (us) + TB6612 motor (%), for the real car.

The strategies publish a bicycle-model Twist. Steering = atan(w * wheelbase / v); with
v ~ 0 the last angle is kept, so a strategy can pre-set the wheels while creeping.
A watchdog stops the motor (and centres nothing) when commands stop arriving.

The speed sent to the motor follows the commanded one with at most accel_limit (m/s^2)
when speeding up and decel_limit when slowing down or reversing (through 0): on the car a
full-power start lifts the front wheels and a hard stop makes the wheels skid
(8 oct 2026). 0 disables a limit. The steering follows the command directly.
/robotek/estop (Bool true) drops the speed to 0 at once (no deceleration ramp).
/robotek/stop (Bool true, second press of the board button) does the same and holds the
motor at 0, ignoring cmd_vel speed, until /robotek/stop false or a restart.

The strategy's angle is the wheel angle. With wheel_table_* (robot.yaml) the servo gets
the angle that really produces it on each side (measured by driving circles: at 25 deg
of servo the wheels reach ~19.7 deg left, ~17.6 deg right), clamped to wheel_limit_deg so
both sides have the same range. Without a table the angle goes to the servo unchanged.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from .mapping import (MotorCalibration, ServoCalibration, motor_pct, rate_limit, servo_for_wheel, servo_us,
                      steer_from_twist)

TICK = .02  # s, motor update period

try:
    from rrc_lite_msgs.msg import SetAckerServoState
except ImportError:  # only installed on the robot (HiWonder image)
    SetAckerServoState = None


class BaseAdapter(Node):
    def __init__(self):
        super().__init__('base_adapter')
        p = {k: self.declare_parameter(k, v).value for k, v in dict(
            wheelbase=.13, servo_topic='/rrc/acker_servo/set_state',
            motor_topic='/motor_vel', servo_duration=.05, watchdog=.3,
            servo_center_us=1500., servo_gain_pos=550./29., servo_gain_neg=500./29., servo_sign=-1.,
            servo_min_us=1000., servo_max_us=2050., steer_limit_deg=25.,
            motor_pct_per_mps=40., motor_min_pct=12., motor_max_pct=100.,
            accel_limit=1., decel_limit=.6,
            wheel_table_servo_deg=[0.], wheel_table_left_deg=[0.], wheel_table_right_deg=[0.],
            wheel_limit_deg=0., servo_measured=True).items()}
        servo_pts = [math.radians(float(x)) for x in p['wheel_table_servo_deg'] if float(x) > 0.]
        n = len(servo_pts)
        left = [math.radians(float(x)) for x in p['wheel_table_left_deg']][-n:] if n else []
        right = [math.radians(float(x)) for x in p['wheel_table_right_deg']][-n:] if n else []
        if n and not (len(left) == len(right) == n):
            raise ValueError('wheel_table_servo_deg, _left_deg and _right_deg must have the same length')
        self.wheel_table = (servo_pts, left, right, math.radians(float(p['wheel_limit_deg'])))
        self.p = p
        if not p['servo_measured']:
            self.get_logger().warn('servo_measured: false -> the steering servo moved and turned 90 deg in the '
                                   'new build; servo_center_us, servo_sign and wheel_table_* are the old ones. '
                                   'Calibrate (README of robotek_hardware, section Calibracion) before driving.')
        self.servo = ServoCalibration(p['servo_center_us'], p['servo_gain_pos'], p['servo_gain_neg'],
                                      p['servo_sign'], p['servo_min_us'], p['servo_max_us'],
                                      p['steer_limit_deg'])
        self.motor = MotorCalibration(p['motor_pct_per_mps'], p['motor_min_pct'], p['motor_max_pct'])
        if SetAckerServoState is None:
            self.get_logger().error('rrc_lite_msgs missing: servo commands disabled.')
            self.servo_pub = None
        else:
            self.servo_pub = self.create_publisher(SetAckerServoState, p['servo_topic'], 10)
        self.motor_pub = self.create_publisher(Float32, p['motor_topic'], 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_subscription(Bool, '/robotek/estop', self.on_estop, 10)
        self.create_subscription(Bool, '/robotek/stop', self.on_stop, 10)
        self.stopped = False              # latched operator stop
        self.steer = 0.
        self.target = self.speed = 0.     # commanded and rate-limited speed (m/s)
        self.last = self.get_clock().now()
        self.create_timer(TICK, self.on_tick)

    def on_cmd(self, msg):
        self.last = self.get_clock().now()
        self.target = 0. if self.stopped else float(msg.linear.x)
        self.steer = steer_from_twist(msg.linear.x, msg.angular.z, self.p['wheelbase'], self.steer)
        servo = servo_for_wheel(self.steer, *self.wheel_table)
        self.send_servo(servo_us(servo, self.servo))

    def on_estop(self, msg):
        if msg.data:
            self.target = self.speed = 0.
            self.motor_pub.publish(Float32(data=0.))

    def on_stop(self, msg):
        self.stopped = bool(msg.data)
        if self.stopped:
            self.target = self.speed = 0.
            self.motor_pub.publish(Float32(data=0.))
            self.get_logger().warning('Operator stop: motor held at 0.')

    def send_servo(self, us):
        if self.servo_pub is not None:
            command = SetAckerServoState()
            command.position = int(round(us))
            command.duration = float(self.p['servo_duration'])
            self.servo_pub.publish(command)

    def on_tick(self):
        if (self.get_clock().now()-self.last).nanoseconds*1e-9 > self.p['watchdog']:
            self.target = 0.               # commands stopped: slow down to 0 (decel_limit)
        if self.stopped:
            self.target = self.speed = 0.
        self.speed = rate_limit(self.speed, self.target, float(self.p['accel_limit']),
                                float(self.p['decel_limit']), TICK)
        if self.target == 0. and abs(self.speed) < 1e-3:
            self.speed = 0.
        self.motor_pub.publish(Float32(data=float(motor_pct(self.speed, self.motor))))


def main(args=None):
    rclpy.init(args=args)
    node = BaseAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():   # on a launch shutdown the context is gone; motor_driver's watchdog stops it
            node.motor_pub.publish(Float32(data=0.))
        node.destroy_node()
        rclpy.try_shutdown()
