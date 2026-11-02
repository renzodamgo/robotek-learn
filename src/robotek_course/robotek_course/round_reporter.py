"""Live report of an Open round on the real car, in the launch terminal (car.launch.py debug:=true).

Works with any strategy: it only listens. It writes, with the time since Start:
  - every change of /robotek/state;
  - the camera's view while the direction is being decided (blue/orange line length, once a second);
  - the direction, from the gyroscope during the first corner (turning left = counter-clockwise);
  - every corner (a TURN that ends) as "corner N/12, lap L", and every finished lap with its time;
  - every blue or orange line driven over (robotek_course.vision.LineCounter), with the totals;
  - the total time when the state is DONE.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Bool, String

from .vision import LineCounter, line_lengths

CORNERS = 12


class RoundReporter(Node):
    def __init__(self):
        super().__init__('round_reporter')
        self.start = None
        self.state = None
        self.corners = 0
        self.lap_start = None
        self.direction = None
        self.turn_yaw = 0.0
        self.imu_time = None
        self.counter = LineCounter()
        self.lines = (0, 0)
        self.view_time = 0.0
        self.bridge = None
        try:
            from cv_bridge import CvBridge
            self.bridge = CvBridge()
            self.create_subscription(Image, '/camera/image', self.on_image, qos_profile_sensor_data)
        except ImportError:
            self.get_logger().warning('cv_bridge missing: no camera report.')
        self.create_subscription(String, '/robotek/state', self.on_state,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Bool, '/start', self.on_start, 10)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.get_logger().info('Round report on (debug): states, direction, corners, laps and lines.')

    def t(self):
        return time.monotonic() - self.start if self.start is not None else 0.0

    def say(self, text):
        self.get_logger().info(f'[{self.t():6.1f} s] {text}')

    def on_start(self, msg):
        if msg.data and self.start is None:
            self.start = self.lap_start = time.monotonic()
            self.say('Start')

    def on_state(self, msg):
        state = msg.data
        if state == self.state:
            return
        previous, self.state = self.state, state
        if state == 'TURN':
            self.turn_yaw = 0.0
        if previous == 'TURN' and state != 'TURN':
            self.corners += 1
            lap = (self.corners - 1) // 4 + 1
            if self.direction is None:
                self.direction = 'antihorario (izquierda)' if self.turn_yaw > 0 else 'horario (derecha)'
                self.say(f'Sentido: {self.direction} ({math.degrees(self.turn_yaw):+.0f} grados en la 1.a esquina)')
            self.say(f'Esquina {self.corners}/{CORNERS} (vuelta {lap}, {math.degrees(self.turn_yaw):+.0f} grados)')
            if self.corners % 4 == 0:
                now = time.monotonic()
                self.say(f'Vuelta {self.corners // 4} completa en {now - self.lap_start:.1f} s')
                self.lap_start = now
        self.say(f'Estado: {state}')
        if state == 'DONE':
            self.say(f'Ronda terminada: {self.corners} esquinas, líneas azul/naranja {self.lines[0]}/{self.lines[1]}, '
                     f'{self.t():.1f} s desde Start')

    def on_imu(self, msg):
        now = time.monotonic()
        if self.imu_time is not None and self.state == 'TURN':
            self.turn_yaw += msg.angular_velocity.z * (now - self.imu_time)
        self.imu_time = now

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        lines = self.counter.update(frame)
        if lines != self.lines:
            colour = 'azul' if lines[0] != self.lines[0] else 'naranja'
            self.lines = lines
            self.say(f'Línea {colour} cruzada (azul {lines[0]}, naranja {lines[1]})')
        if self.state in ('CALIBRATE', 'DETECT_DIRECTION') and time.monotonic() - self.view_time > 1.0:
            self.view_time = time.monotonic()
            blue, orange = line_lengths(frame)
            self.say(f'Cámara: azul {blue:.0f} px, naranja {orange:.0f} px')


def main(args=None):
    rclpy.init(args=args)
    node = RoundReporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
