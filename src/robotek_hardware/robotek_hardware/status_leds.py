"""Shows the car's state on the two RGBs of the RRC Lite (the board buzzer is burnt out).

Colour code in status_colors.py: green = ready for the button, blue = pressed (the round runs), red blinking =
battery low. Reads only the strategy state (/robotek/state) and the battery stop of battery_guard
(/robotek/battery_low). Display only: nothing here changes how the car drives, and it takes no input from the team
(rules 9.9-9.11).

  ros2 run robotek_hardware status_leds --ros-args -p level:=2
"""
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from .status_colors import OFF, READY, scaled, status_colors

try:
    from rrc_lite_msgs.msg import RGBState, RGBStates
except ImportError:  # only installed on the robot (HiWonder image)
    RGBState = RGBStates = None

RESEND = 1.0   # s
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class StatusLeds(Node):
    def __init__(self):
        super().__init__('status_leds')
        p = {name: self.declare_parameter(name, value).value for name, value in (
            ('level', 2),             # of 255: very dim, so the colour does not reflect into the camera
            ('rate', 4.0))}           # Hz: updates and blink (blinks at rate/2)
        self.level = int(p['level'])
        self.state, self.battery_low, self.blink, self.last = '', False, False, None
        self.last_sent = 0.0
        if RGBStates is None:
            self.get_logger().error('rrc_lite_msgs missing: no RGB output.')
            return
        self.rgb = self.create_publisher(RGBStates, '/rrc/set_rgb', 10)
        self.create_subscription(String, '/robotek/state', self.on_state, LATCHED)
        self.create_subscription(Bool, '/robotek/battery_low', self.on_battery_low, LATCHED)
        self.create_timer(1./float(p['rate']), self.update)
        self.get_logger().info(f'Status RGBs at level {self.level}/255: green blinking = ready, blue = running.')

    def on_state(self, msg):
        if msg.data != self.state:
            if msg.data in READY:      # green in the terminal too, like the LEDs (11 nov 2026)
                self.get_logger().info(f'\033[1;32mREADY ({msg.data}): press the button to start\033[0m')
            elif self.state in READY:
                self.get_logger().info(f'\033[1;34mRUNNING ({msg.data})\033[0m')
        self.state = msg.data

    def on_battery_low(self, msg):
        self.battery_low = self.battery_low or msg.data   # latched, like battery_guard

    def update(self):
        self.blink = not self.blink
        self.show(status_colors(self.state, self.battery_low, self.blink))

    def show(self, colors, force=False):
        colors = tuple(scaled(c, self.level) for c in colors)
        now = time.monotonic()
        # Changes at once, and the same colour again every RESEND s: a single lost message left the LEDs green for a
        # whole round (4 nov 2026, tests 25-26), and a board reset (the brake) turns them off.
        if colors == self.last and not force and now - self.last_sent < RESEND:
            return
        self.last, self.last_sent = colors, now
        msg = RGBStates()
        for index, (r, g, b) in enumerate(colors, start=1):
            state = RGBState()
            state.index, state.red, state.green, state.blue = index, r, g, b
            msg.states.append(state)
        self.rgb.publish(msg)


def main(args=None):
    # Own signal handling (like the strategies) so the RGBs can be turned off before rclpy
    # shuts down; with the default handler Ctrl+C left them lit (9 oct 2026).
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = StatusLeds()
    stop = []
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stop.append(True))
    try:
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if RGBStates is not None and rclpy.ok():
            node.show((OFF, OFF), force=True)
            time.sleep(0.2)   # let the message leave before the node goes away
        node.destroy_node()
        rclpy.try_shutdown()
