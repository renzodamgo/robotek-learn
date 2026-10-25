"""Shows the car's state on the two RGBs of the RRC Lite (the board buzzer is burnt out).

Colour code in status_colors.py. Reads the strategy state (/robotek/state), the sensors
(LiDAR, IMU, camera: fresh if a message arrived in the last `max_age` s), and the battery stop
of battery_guard (/robotek/battery_low). Display only: nothing here changes how the car drives,
and it takes no input from the team (rules 9.9-9.11).

  ros2 run robotek_hardware status_leds --ros-args -p level:=2
"""
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Bool, String

from .status_colors import OFF, scaled, status_colors

try:
    from rrc_lite_msgs.msg import RGBState, RGBStates
except ImportError:  # only installed on the robot (HiWonder image)
    RGBState = RGBStates = None

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class StatusLeds(Node):
    def __init__(self):
        super().__init__('status_leds')
        p = {name: self.declare_parameter(name, value).value for name, value in (
            ('level', 2),             # of 255: very dim, so the colour does not reflect into the camera
            ('max_age', 1.0),         # s without data = sensor missing
            ('boot_grace', 15.0),     # s after launch with everything off while drivers start
            ('rate', 4.0),            # Hz: updates and blink (blinks at rate/2)
            ('camera_required', True),
            ('scan_topic', '/robotek/scan'),
            ('imu_topic', '/rrc/imu_raw'),
            ('camera_topic', '/usb_cam/image_raw'))}
        self.level, self.max_age = int(p['level']), float(p['max_age'])
        self.camera_required = bool(p['camera_required'])
        self.boot_until = time.monotonic()+float(p['boot_grace'])
        self.seen = {'lidar': None, 'imu': None, 'camera': None}
        self.state, self.battery_low, self.blink, self.last = '', False, False, None
        if RGBStates is None:
            self.get_logger().error('rrc_lite_msgs missing: no RGB output.')
            return
        self.rgb = self.create_publisher(RGBStates, '/rrc/set_rgb', 10)
        sensor = qos_profile_sensor_data
        self.create_subscription(LaserScan, p['scan_topic'], lambda _: self.mark('lidar'), sensor)
        self.create_subscription(Imu, p['imu_topic'], lambda _: self.mark('imu'), sensor)
        if self.camera_required:
            self.create_subscription(Image, p['camera_topic'], lambda _: self.mark('camera'), sensor)
        self.create_subscription(String, '/robotek/state', self.on_state, LATCHED)
        self.create_subscription(Bool, '/robotek/battery_low', self.on_battery_low, LATCHED)
        self.create_timer(1./float(p['rate']), self.update)
        self.get_logger().info(f'Status RGBs at level {self.level}/255.')

    def mark(self, sensor):
        self.seen[sensor] = time.monotonic()

    def ok(self, sensor, now):
        if sensor == 'camera' and not self.camera_required:
            return True
        return self.seen[sensor] is not None and now-self.seen[sensor] < self.max_age

    def on_state(self, msg):
        self.state = msg.data

    def on_battery_low(self, msg):
        self.battery_low = self.battery_low or msg.data   # latched, like battery_guard

    def update(self):
        now = time.monotonic()
        self.blink = not self.blink
        colors = status_colors(self.state, self.ok('lidar', now), self.ok('imu', now),
                               self.ok('camera', now), self.battery_low, now < self.boot_until,
                               self.blink)
        self.show(colors)

    def show(self, colors, force=False):
        colors = tuple(scaled(c, self.level) for c in colors)
        if colors == self.last and not force:
            return   # the board keeps the colour: only send changes
        self.last = colors
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
