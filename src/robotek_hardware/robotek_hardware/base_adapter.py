"""/cmd_vel (Twist) -> RRC Lite servo (us) + drive motor, for the real car.

The drive motor goes one of two ways (motor_backend in robot.yaml):
  - 'tb6612' (default): a DC motor on the TB6612 bridge, as a % on /motor_vel for motor_driver;
  - 'rrc': an encoder motor on a motor port of the RRC Lite (rrc_motor_port, e.g. 4 = M4), as revolutions
    per second on /rrc/set_motor; the board holds that speed with the encoder. motor_rps_per_mps must be
    measured on the car (motor rps per m/s); until it is (0), the motor stays at 0.

Wall guard (wall_stop_m > 0): with the nearest LiDAR return within +-wall_half_deg of straight ahead
(/robotek/scan, 0 rad ahead) nearer than wall_slow_m, the forward speed is scaled down linearly, to 0 at
wall_stop_m. The speed still follows decel_limit, so the car slows down smoothly instead of braking hard.
Reversing is not limited. Without scans (none in the last 0.5 s) the guard does nothing.

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

Brake (rrc backend, brake_rps > 0): the RRC Lite did not hold 0 with the 310 motor (it ran on, sped up or rolled
back, 4 nov 2026). When the speed goes to 0 after moving (target 0, the board button, estop or the watchdog),
the ramp is skipped and the motor gets +-brake_rps alternating at brake_hz for brake_s (reverse first), then 0
(mapping.DitherBrake): on the floor that stopped the car ~12 cm after the order, 3 times out of 3.

The strategy's angle is the wheel angle. With wheel_table_* (robot.yaml) the servo gets
the angle that really produces it on each side (measured by driving circles: at 25 deg
of servo the wheels reach ~19.7 deg left, ~17.6 deg right), clamped to wheel_limit_deg so
both sides have the same range. Without a table the angle goes to the servo unchanged.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float32, String

from .mapping import (DitherBrake, MotorCalibration, ServoCalibration, front_distance, motor_pct, motor_rps, rate_limit,
                      servo_for_wheel, servo_us, steer_from_twist, wall_speed_factor)

TICK = .02  # s, motor update period

try:
    from rrc_lite_msgs.msg import MotorsState, MotorState, SetAckerServoState
except ImportError:  # only installed on the robot (HiWonder image)
    MotorsState = MotorState = SetAckerServoState = None


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
            wheel_limit_deg=0., servo_measured=True,
            motor_backend='tb6612', rrc_motor_topic='/rrc/set_motor', rrc_motor_port=4, rrc_motor_sign=1.,
            motor_rps_per_mps=0., motor_max_rps=0.,
            brake_rps=0., brake_hz=10., brake_s=1.5, brake_min_rps=1., brake_reset=False,
            brake_reset_states='FINISH,DONE,OPERATOR_STOP,DIRECTION_FAULT',
            wall_stop_m=0., wall_slow_m=0.6, wall_half_deg=15., scan_topic='/robotek/scan').items()}
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
        self.backend = str(p['motor_backend'])
        if self.backend not in ('tb6612', 'rrc'):
            raise ValueError(f"motor_backend must be 'tb6612' or 'rrc', got {self.backend!r}")
        if self.backend == 'rrc':
            if MotorsState is None:
                raise RuntimeError("motor_backend 'rrc' needs rrc_lite_msgs")
            self.motor_pub = self.create_publisher(MotorsState, p['rrc_motor_topic'], 10)
            if float(p['motor_rps_per_mps']) <= 0.:
                self.get_logger().error('motor_backend rrc with motor_rps_per_mps <= 0: not measured yet, the motor '
                                        'stays at 0 (robot.yaml).')
            self.get_logger().info(f"Drive motor: RRC Lite port M{int(p['rrc_motor_port'])}, "
                                   f"{float(p['motor_rps_per_mps']):.2f} rps per m/s.")
        else:
            self.motor_pub = self.create_publisher(Float32, p['motor_topic'], 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.front = math.inf             # nearest obstacle ahead (m), from the LiDAR
        self.front_time = None
        if float(p['wall_stop_m']) > 0.:
            self.create_subscription(LaserScan, p['scan_topic'], self.on_scan, qos_profile_sensor_data)
            self.get_logger().info(f"Wall guard: slows down from {float(p['wall_slow_m']):.2f} m, "
                                   f"0 at {float(p['wall_stop_m']):.2f} m (+-{float(p['wall_half_deg']):.0f} deg).")
        self.create_subscription(Bool, '/robotek/estop', self.on_estop, 10)
        self.create_subscription(Bool, '/robotek/stop', self.on_stop, 10)
        self.stopped = False              # latched operator stop
        self.steer = 0.
        self.target = self.speed = 0.     # commanded and rate-limited speed (m/s)
        self.brake = DitherBrake(p['brake_rps'], p['brake_hz'], p['brake_s'], p['brake_min_rps']) \
            if self.backend == 'rrc' else DitherBrake()
        # brake_reset: instead of the back-and-forth, reset the board (rrc_driver /rrc/reset, RTS = RESET): the stop
        # the 310 motor obeyed every time (4 nov 2026). The board is set up again ~2.5 s later.
        self.brake_reset = bool(p['brake_reset']) and self.brake.enabled
        self.reset_pub = self.create_publisher(Empty, '/rrc/reset', 1) if self.brake_reset else None
        self.reset_sent = False
        # Only at the end (4 nov 2026, test 26): a mid-round stop (the strategy braking for a stale IMU) reset the
        # board, which cut the IMU for 2.5 s more and kept the car stopped ~4 s. The reset now needs one of these
        # /robotek/state values (FINISH: the final stop starts there, DONE comes after the car is at rest), the
        # board button stop or the watchdog; any other stop is a plain 0. '' = every stop after moving (before).
        self.reset_states = {s.strip() for s in str(p['brake_reset_states']).split(',') if s.strip()}
        self.state = ''
        self.create_subscription(String, '/robotek/state', self.on_state,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if self.brake_reset:
            self.get_logger().info('Brake: board reset (/rrc/reset) when stopping after moving'
                                   + (f" in {', '.join(sorted(self.reset_states))}, the button stop or the watchdog;"
                                      ' other stops send 0.' if self.reset_states else '.'))
        elif self.brake.enabled:
            self.get_logger().info(f"Brake: +-{self.brake.amp:.1f} rps at {self.brake.hz:.0f} Hz for "
                                   f"{self.brake.duration:.1f} s when stopping after moving.")
        self.last = self.get_clock().now()
        self.create_timer(TICK, self.on_tick)

    def on_state(self, msg):
        self.state = msg.data

    def final_stop(self):
        """A stop the board reset may do: the end of the round, the button stop or lost commands."""
        if not self.reset_states or self.stopped or self.state in self.reset_states:
            return True
        return (self.get_clock().now()-self.last).nanoseconds*1e-9 > self.p['watchdog']

    def on_cmd(self, msg):
        self.last = self.get_clock().now()
        self.target = 0. if self.stopped else float(msg.linear.x)
        self.steer = steer_from_twist(msg.linear.x, msg.angular.z, self.p['wheelbase'], self.steer)
        servo = servo_for_wheel(self.steer, *self.wheel_table)
        self.send_servo(servo_us(servo, self.servo))

    def on_scan(self, msg):
        front = front_distance(msg.ranges, msg.angle_min, msg.angle_increment, float(self.p['wall_half_deg']))
        # Very close the LiDAR returns nothing (scan_adapter drops ranges under min_valid_range, ~0.12 m): an
        # obstacle that was near and vanishes is still there, so keep the last near reading until a finite one.
        if math.isfinite(front) or self.front >= float(self.p['wall_slow_m']):
            self.front = front
        self.front_time = self.get_clock().now()

    def top_speed(self):
        """Fastest the motor can really go (m/s) with the configured cap, inf if uncapped."""
        p = self.p
        if self.backend == 'rrc':
            if float(p['motor_max_rps']) > 0. and float(p['motor_rps_per_mps']) > 0.:
                return float(p['motor_max_rps'])/float(p['motor_rps_per_mps'])
            return math.inf
        if float(p['motor_pct_per_mps']) > 0.:
            return max(0., (float(p['motor_max_pct'])-float(p['motor_min_pct']))/float(p['motor_pct_per_mps']))
        return math.inf

    def guarded(self, target):
        """target capped at the motor's real top speed, then limited by the wall guard (forward only, and only
        with a recent scan). Capping first matters: the strategy asks 1.28 m/s, the car does ~0.2, and scaling
        1.28 down would not slow the car until the very last centimetres (1 nov 2026, it hit a wall)."""
        top = self.top_speed()
        target = max(-top, min(top, target))
        if target <= 0. or self.front_time is None or \
                (self.get_clock().now()-self.front_time).nanoseconds*1e-9 > .5:
            return target
        return target*wall_speed_factor(self.front, float(self.p['wall_stop_m']), float(self.p['wall_slow_m']))

    def on_estop(self, msg):
        if msg.data:
            self.target = self.speed = 0.
            self.send_speed(0.)

    def on_stop(self, msg):
        self.stopped = bool(msg.data)
        if self.stopped:
            self.target = self.speed = 0.
            self.send_speed(0.)
            self.get_logger().warning('Operator stop: motor held at 0.')

    def send_speed(self, speed):
        """Car speed (m/s) to the drive motor of the chosen backend."""
        p = self.p
        if self.backend == 'rrc':
            rps_per_mps = float(p['motor_rps_per_mps'])
            rps = motor_rps(speed, rps_per_mps, float(p['motor_max_rps']), float(p['rrc_motor_sign'])) \
                if rps_per_mps > 0. else 0.
            rps = self.brake.rps(self.get_clock().now().nanoseconds * 1e-9, rps)
            if self.brake_reset:
                if self.brake.start is not None:      # braking: one board reset (at the end), else plain 0
                    if not self.reset_sent and self.final_stop():
                        self.reset_pub.publish(Empty())
                        self.reset_sent = True
                        self.get_logger().warning('Stopping after moving: board reset.')
                    rps = 0.
                else:
                    self.reset_sent = False
            self.motor_pub.publish(MotorsState(data=[MotorState(id=int(p['rrc_motor_port']), rps=float(rps))]))
        else:
            self.motor_pub.publish(Float32(data=float(motor_pct(speed, self.motor))))

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
        if self.brake.enabled and self.target == 0.:
            self.speed = 0.                # no ramp: the brake takes over from the last real speed
        self.speed = rate_limit(self.speed, self.guarded(self.target), float(self.p['accel_limit']),
                                float(self.p['decel_limit']), TICK)
        if self.target == 0. and abs(self.speed) < 1e-3:
            self.speed = 0.
        self.send_speed(self.speed)


def main(args=None):
    rclpy.init(args=args)
    node = BaseAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():   # on a launch shutdown the context is gone; motor_driver's / rrc_driver's watchdog stops it
            node.send_speed(0.)
        node.destroy_node()
        rclpy.try_shutdown()
