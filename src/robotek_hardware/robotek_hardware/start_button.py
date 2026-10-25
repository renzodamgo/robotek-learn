"""RRC Lite button -> /start (std_msgs/Bool), with a beep. Press edges only (state 1 or 5),
debounced like the team's robot_node (1.5 s).

Any press after the first one publishes /robotek/stop (Bool true, two beeps): base_adapter
and motor_driver hold the motor at 0 and the Open strategy stops for good. Restart the
launch to run again. (8 Oct 2026: the car hit a wall and could not be stopped.)"""
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
        if ButtonState is None:
            self.get_logger().error('rrc_lite_msgs missing: publish /start by hand.')
            return
        self.buzzer = self.create_publisher(BuzzerState, '/rrc/set_buzzer', 10)
        self.create_subscription(ButtonState, '/rrc/button', self.on_button, 10)
        self.get_logger().info('Press the button to start; press it again to stop.')

    def on_button(self, msg):
        if msg.state not in PRESS_STATES:
            return
        now = self.get_clock().now().nanoseconds*1e-9
        if self.last is not None and now-self.last < DEBOUNCE:
            return
        self.last = now
        if self.started:
            self.stop_pub.publish(Bool(data=True))
            self.beep(repeat=2)
            self.get_logger().warning('Stop (button pressed again).')
            return
        self.started = True
        self.pub.publish(Bool(data=True))
        self.beep(repeat=1)
        self.get_logger().info('Start.')

    def beep(self, repeat):
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
