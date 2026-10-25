"""TB6612FNG driver (channel A) for the Raspberry Pi 5, with reverse.

Based on the team's first motor driver: std_msgs/Float32 on
/motor_vel, now -100..100 % (negative = reverse, needed for parking and recovery).
On 0 it stops softly (brake_mode 'ramp', default): the duty falls to 0 over
stop_ramp_s (0.6 s; 0.3 s still skidded a little, 8 oct 2026) in the same direction, then the bridge is left free (coast). The old
brake_mode 'reverse' drives against the last direction for BRAKE_TIME; on the car it
made the wheels skid and the deceleration too abrupt (8 oct 2026). A direction change
passes through the stop first. A watchdog stops the motor (same soft stop) when commands
stop arriving. /robotek/estop (std_msgs/Bool true) cuts the motor at once, without the
ramp, and ignores commands until false is published or 1 s passes without true.
/robotek/stop (Bool true, second press of the board button) stops the motor and ignores
every command until /robotek/stop false or a restart (latched operator stop).
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

try:
    import lgpio
except ImportError:
    lgpio = None

BRAKE_TIME = .12  # s, reverse brake (brake_mode 'reverse')
TICK = .02        # s


class Tb6612Motor(Node):
    def __init__(self):
        super().__init__('tb6612fng_motor')
        p = {k: self.declare_parameter(k, v).value for k, v in dict(
            gpiochip=4, pwm_a=12, ain1=5, ain2=6, stby=13, pwm_hz=1000, watchdog_s=.30,
            brake_mode='ramp', stop_ramp_s=.6).items()}
        self.brake_mode = str(p['brake_mode'])
        self.stop_ramp = max(TICK, float(p['stop_ramp_s']))
        self.ramp_from = None
        self.chip = int(p['gpiochip'])
        self.pins = {name: int(p[name]) for name in ('pwm_a', 'ain1', 'ain2', 'stby')}
        self.hz = int(p['pwm_hz'])
        self.watchdog_s = float(p['watchdog_s'])
        self.gpio = None
        self.last_command = time.monotonic()
        self.last_pct = 0.
        self.braking_until = None
        if lgpio is None:
            self.get_logger().error('lgpio missing (sudo apt install python3-lgpio): motor disabled.')
        elif not self.open_gpio():
            # Not on a Raspberry Pi, or the user cannot open /dev/gpiochipN: keep retrying, never crash.
            self.create_timer(5., self.retry_gpio)
        self.create_subscription(Float32, '/motor_vel', self.on_command, 10)
        self.create_subscription(Bool, '/robotek/estop', self.on_estop, 10)
        self.create_subscription(Bool, '/robotek/stop', self.on_stop, 10)
        self.estop_until = 0.
        self.stopped = False
        self.create_timer(TICK, self.on_watchdog)

    def open_gpio(self):
        """Claim the bridge pins, all low (motor stopped). False, with a clear log, if the GPIO chip is not usable."""
        handle = None
        try:
            handle = lgpio.gpiochip_open(self.chip)
            for pin in self.pins.values():
                lgpio.gpio_claim_output(handle, pin, 0)
        except Exception as error:   # lgpio.error: no such chip, permission denied, pin busy
            if handle is not None:
                try:
                    lgpio.gpiochip_close(handle)
                except Exception:   # noqa: BLE001
                    pass
            self.get_logger().warn(
                f'GPIO chip {self.chip} not usable ({error}): motor disabled, retrying every 5 s. '
                'Is this a Raspberry Pi 5 with python3-lgpio, and is your user in the "gpio" group '
                '(or is another program holding the pins)? The motor does NOT move without this.',
                throttle_duration_sec=60.)
            return False
        self.gpio = handle
        self.stop_motor()
        self.get_logger().info('TB6612FNG ready (forward and reverse).')
        return True

    def retry_gpio(self):
        if self.gpio is None:
            self.open_gpio()

    def stop_motor(self):
        if self.gpio is None:
            return
        lgpio.tx_pwm(self.gpio, self.pins['pwm_a'], self.hz, 0)
        for name in ('ain1', 'ain2', 'stby'):
            lgpio.gpio_write(self.gpio, self.pins[name], 0)
        self.last_pct = 0.
        self.braking_until = None
        self.ramp_from = None

    def drive(self, pct):
        """pct in -100..100; the sign selects the bridge direction."""
        forward = pct > 0.
        lgpio.gpio_write(self.gpio, self.pins['ain1'], 1 if forward else 0)
        lgpio.gpio_write(self.gpio, self.pins['ain2'], 0 if forward else 1)
        lgpio.gpio_write(self.gpio, self.pins['stby'], 1)
        # Always PWM (duty 0-100): once tx_pwm runs on a pin, lgpio ignores gpio_write on it,
        # so a plain "write 1" at 100 % left the motor at the last duty (0 after a stop).
        # Found on the car 8 oct 2026: 50 % pulses moved it, 100 % pulses did not.
        lgpio.tx_pwm(self.gpio, self.pins['pwm_a'], self.hz, min(100., abs(pct)))
        self.last_pct = pct

    def brake(self):
        """Stop from the last command: soft ramp to 0 (default) or a short reverse pulse."""
        if self.brake_mode == 'reverse':
            self.drive(-self.last_pct)
            self.braking_until = time.monotonic()+BRAKE_TIME
            self.last_pct = 0.
            return
        self.ramp_from = self.last_pct
        self.ramp_start = time.monotonic()
        self.braking_until = self.ramp_start+self.stop_ramp

    def on_estop(self, msg):
        if msg.data:
            self.estop_until = time.monotonic()+1.
            self.stop_motor()
        else:
            self.estop_until = 0.

    def on_stop(self, msg):
        if msg.data and not self.stopped:
            self.stop_motor()
            self.get_logger().warning('Operator stop: ignoring motor commands.')
        self.stopped = bool(msg.data)

    def on_command(self, msg):
        if self.stopped:
            return   # the stop already started; without commands the watchdog keeps it stopped
        if time.monotonic() < self.estop_until:
            self.stop_motor()
            return
        try:
            pct = max(-100., min(100., float(msg.data)))
        except (TypeError, ValueError):
            pct = 0.
        self.last_command = time.monotonic()
        if self.gpio is None:
            return
        if self.braking_until is not None:
            return  # finish the brake first
        reversing = pct*self.last_pct < 0.
        if pct == 0. or reversing:
            if self.last_pct != 0.:
                self.brake()
            else:
                self.stop_motor()
            return
        self.drive(pct)

    def on_watchdog(self):
        now = time.monotonic()
        if self.braking_until is not None:
            if now >= self.braking_until:
                self.stop_motor()
            elif self.ramp_from is not None:
                self.drive(self.ramp_from*(1.-(now-self.ramp_start)/self.stop_ramp))
            return
        if now-self.last_command > self.watchdog_s:
            if self.last_pct != 0.:
                self.brake()
            else:
                self.stop_motor()

    def destroy_node(self):
        self.stop_motor()
        if self.gpio is not None:
            lgpio.gpiochip_close(self.gpio)
            self.gpio = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Tb6612Motor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
