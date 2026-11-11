"""Start button -> /start (std_msgs/Bool), with a beep. The press comes from the push button on the Pi's GPIO 16
(/robotek/gpio_button, gpio_button node; ``gpio_button`` true, the default since 10 nov 2026) and/or the RRC Lite's
button (/rrc/button, press edges only, state 1 or 5; ``board_button``, off by default since 10 nov 2026). Debounced
like the team's robot_node (1.5 s).

Any press after the first one publishes /robotek/stop (Bool true, two beeps): base_adapter
and motor_driver hold the motor at 0 and the Open strategy stops for good. Restart the
launch to run again. (8 Oct 2026: the car hit a wall and could not be stopped.)
Presses in the first stop_lockout seconds (default 5) after the start are ignored: on 4 nov 2026 a start pressed twice
by mistake stopped the round before the car moved.

auto_start_delay > 0 (car.launch button:=false, for quick tests): the node starts by itself that many seconds after it
comes up, as if the button had been pressed, so the FIRST real press is already the stop."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    from rrc_lite_msgs.msg import BuzzerState, ButtonState
except ImportError:  # only installed on the robot (HiWonder image)
    BuzzerState = ButtonState = None

PRESS_STATES = (1, 5)
DEBOUNCE = 1.5  # s


class StartButton(Node):
    def __init__(self):
        super().__init__('start_button')
        self.pub = self.create_publisher(Bool, 'start', 10)
        self.stop_pub = self.create_publisher(Bool, '/robotek/stop', 10)
        self.last = None
        self.started = False
        self.started_at = None
        self.stop_lockout = float(self.declare_parameter('stop_lockout', 5.0).value)
        board = bool(self.declare_parameter('board_button', False).value)
        gpio = bool(self.declare_parameter('gpio_button', True).value)
        if gpio:
            self.create_subscription(Bool, '/robotek/gpio_button', self.on_gpio, 10)
        delay = float(self.declare_parameter('auto_start_delay', 0.0).value)
        if delay > 0.:
            self.auto_timer = self.create_timer(delay, self.auto_start)
            self.auto_count = 0
            self.get_logger().warning(f'No button: the car starts by itself in {delay:.0f} s; press the button to stop.')
        if ButtonState is None:
            self.get_logger().error('rrc_lite_msgs missing: publish /start by hand.')
            return
        self.buzzer = self.create_publisher(BuzzerState, '/rrc/set_buzzer', 10)
        if board:
            self.create_subscription(ButtonState, '/rrc/button', self.on_button, 10)
        self.get_logger().info('Press the %s to start; press it again to stop.' % ' or '.join(
            name for name, on in (('GPIO 16 button', gpio), ("board's button", board)) if on))

    def auto_start(self):
        """Start without the button; /start goes out three times, 0.5 s apart, in case the strategy is still waking up."""
        if self.auto_count == 0:
            self.auto_timer.cancel()
            self.auto_timer = self.create_timer(0.5, self.auto_start)
            if self.started:
                self.auto_timer.cancel()
                return
            self.started = True
            self.started_at = self.get_clock().now().nanoseconds*1e-9
            self.get_logger().info('Start (no button).')
        self.pub.publish(Bool(data=True))
        self.auto_count += 1
        if self.auto_count >= 3:
            self.auto_timer.cancel()

    def on_button(self, msg):
        if msg.state in PRESS_STATES:
            self.press()

    def on_gpio(self, msg):
        if msg.data:                 # True on each debounced press
            self.press()

    def press(self):
        now = self.get_clock().now().nanoseconds*1e-9
        if self.last is not None and now-self.last < DEBOUNCE:
            return
        self.last = now
        if self.started and self.started_at is not None and now-self.started_at < self.stop_lockout:
            self.get_logger().warning(f'Press ignored: {now-self.started_at:.1f} s after the start '
                                      f'(a stop needs {self.stop_lockout:.0f} s).')
            return
        if self.started:
            self.stop_pub.publish(Bool(data=True))
            self.beep(repeat=2)
            self.get_logger().warning('Stop (button pressed again).')
            return
        self.started = True
        self.started_at = now
        self.pub.publish(Bool(data=True))
        self.beep(repeat=1)
        self.get_logger().info('Start.')

    def beep(self, repeat):
        if BuzzerState is None:
            return
        beep = BuzzerState()
        beep.freq, beep.on_time, beep.off_time, beep.repeat = 880, .1, .1, repeat
        self.buzzer.publish(beep)


def main(args=None):
    rclpy.init(args=args)
    node = StartButton()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
