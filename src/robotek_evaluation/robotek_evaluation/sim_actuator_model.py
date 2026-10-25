"""Simulation-only actuator model between a strategy and the Gazebo Ackermann plugin.

The gz-sim 8 AckermannSteering plugin applies ``max_acceleration`` to the yaw-rate
command too, so with 2.0 the steering of the simulated car builds up at ~2 rad/s^2
(~16 deg/s at 0.95 m/s), far slower than a hobby servo. With this node the plugin
limit is raised and the actuators are modelled explicitly:

* drive: speed slews at ``accel_max`` / ``decel_max`` (m/s^2);
* steering: dead time ``steer_delay`` (s), slew ``steer_rate`` (rad/s) and a first-order
  lag ``steer_tau`` (s), clamped to ``steer_limit``.

Optional terms measured on the real car (real_car_v1.yaml, 8 oct 2026); their defaults
leave the model exactly as above:

* ``drive_delay`` (s) dead time and ``drive_tau`` (s) first-order lag of the motor after
  the slew (the slew plays base_adapter's accel/decel limits);
* ``drive_command_limit`` (m/s): saturate the ramped motor command at full PWM
  before mapping to real speed; zero disables it (legacy model);
* ``speed_table_cmd`` / ``speed_table_real`` (m/s): the speed the car really reaches for
  each commanded speed (the motor calibration is only exact up to ~0.3 m/s), piecewise
  linear, odd-symmetric, extrapolated with the last slope; empty = identity;
* ``steer_deadband`` (rad) and ``steer_gain``: the wheels turn
  sign(s) * max(0, |s| - deadband) * gain for a servo angle s;
* ``steer_table_cmd`` with ``steer_table_left`` / ``steer_table_right`` (rad): the wheel
  angle really reached for each servo angle, per side, piecewise linear from 0 (replaces
  deadband and gain when given). Measured by driving circles (8 oct 2026).

Input: /cmd_vel_raw (strategy). Output: /cmd_vel (plugin), with
angular.z = v * tan(steer) / wheelbase. Never used on the real robot.
"""
from collections import deque
import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from .sim_math import slew, table_speed, wheel_angle  # noqa: F401  (re-exported: the tests import them from here)


class SimActuatorModel(Node):
    def __init__(self):
        super().__init__('sim_actuator_model')
        p = {k: self.declare_parameter(k, v).value for k, v in dict(
            rate_hz=100.0, wheelbase=0.13, steer_limit=0.4363323129985824,
            steer_delay=0.03, steer_rate=4.4, steer_tau=0.03,
            accel_max=2.0, decel_max=2.0,
            drive_delay=0.0, drive_tau=0.0, drive_command_limit=0.0, drive_deadband=0.0,
            speed_table_cmd=[0.0], speed_table_real=[0.0],
            steer_deadband=0.0, steer_gain=1.0,
            steer_table_cmd=[0.0], steer_table_left=[0.0], steer_table_right=[0.0]).items()}
        self.p = p
        cmd = [float(x) for x in p['speed_table_cmd'] if float(x) > 0.]
        real = [float(x) for x in p['speed_table_real']][-len(cmd):] if cmd else []
        if len(cmd) != len(real) or any(b <= a for a, b in zip(cmd, cmd[1:])):
            raise ValueError('speed_table_cmd must grow and match speed_table_real')
        self.table = (cmd, real)
        scmd = [float(x) for x in p['steer_table_cmd'] if float(x) > 0.]
        left = [float(x) for x in p['steer_table_left']][-len(scmd):] if scmd else []
        right = [float(x) for x in p['steer_table_right']][-len(scmd):] if scmd else []
        if scmd and not (len(scmd) == len(left) == len(right)):
            raise ValueError('steer_table_cmd, steer_table_left and steer_table_right must match')
        self.steer_table = (scmd, left, right)
        self.v_cmd = self.steer_cmd = 0.
        self.v = self.steer = 0.
        self.v_out = 0.                 # after drive delay, table and lag
        self.queue = deque()
        self.drive_queue = deque()
        self.last = None
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel_raw', self.on_cmd, 10)
        self.create_timer(1.0/p['rate_hz'], self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def on_cmd(self, msg):
        self.v_cmd = msg.linear.x
        if abs(msg.linear.x) > 1e-3:
            self.steer_cmd = math.atan(msg.angular.z*self.p['wheelbase']/msg.linear.x)
        self.queue.append((self.now(), self.steer_cmd))

    def wheel(self, servo):
        cmd, left, right = getattr(self, 'steer_table', ([], [], []))
        if cmd:
            return table_speed(servo, cmd, left if servo >= 0. else right)
        return wheel_angle(servo, self.p['steer_deadband'], self.p['steer_gain'])

    def drive_output(self, now, dt):
        """Motor after the slew: dead time, real-speed table, first-order lag."""
        p = self.p
        v = self.v
        if p['drive_delay'] > 0.:
            self.drive_queue.append((now, v))
            v = None
            while self.drive_queue and now-self.drive_queue[0][0] >= p['drive_delay']:
                v = self.drive_queue.popleft()[1]
            if v is None:
                v = self.delayed_v if hasattr(self, 'delayed_v') else 0.
            self.delayed_v = v
        # base_adapter ramps its speed command, then clamps motor_pct to 100 %.
        drive_limit = p.get('drive_command_limit', 0.0)
        if drive_limit > 0.0:
            v = max(-drive_limit, min(drive_limit, v))
        if abs(v) < p.get('drive_deadband', 0.0):
            v = 0.0
        v = table_speed(v, *self.table)
        if p['drive_tau'] > 0.:
            self.v_out += (v-self.v_out)*min(1., dt/p['drive_tau'])
        else:
            self.v_out = v
        return self.v_out

    def tick(self):
        p, now = self.p, self.now()
        dt = 0. if self.last is None else max(0., min(.05, now-self.last))
        self.last = now
        # Steering target after the dead time.
        target = None
        while self.queue and now-self.queue[0][0] >= p['steer_delay']:
            target = self.queue.popleft()[1]
        if target is not None:
            self.target = max(-p['steer_limit'], min(p['steer_limit'], target))
        target = getattr(self, 'target', 0.)
        change = target-self.steer
        if p['steer_tau'] > 0.:
            change *= min(1., dt/p['steer_tau'])
        step = p['steer_rate']*dt
        self.steer += max(-step, min(step, change))
        # Drive speed slew (plays base_adapter's accel/decel limits), then the motor.
        self.v = slew(self.v, self.v_cmd, p['accel_max'], p['decel_max'], dt)
        v = self.drive_output(now, dt)
        out = Twist()
        out.linear.x = v
        out.angular.z = v*math.tan(self.wheel(self.steer))/p['wheelbase']
        self.pub.publish(out)


def main():
    rclpy.init()
    node = SimActuatorModel()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
