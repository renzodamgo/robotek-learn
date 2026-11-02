"""Drive the car with the keyboard (step 0): w/s speed, a/d steering, space stops, q quits.

    ros2 run robotek_course keyboard_drive
    ros2 run robotek_course keyboard_drive --ros-args -p hold:=true     # real car: move it to the start

hold:=true (to place the real car, with car.launch.py teleop:=true): the car moves only while w (forward) or s (back)
is held, at hold_speed m/s, and stops hold_timeout s after the key is released (the terminal repeats a held key);
a/d steer, x centres the wheels. Leaving (q or Ctrl+C) sends zero speed.

Publishes /cmd_vel_raw like a strategy would: linear.x is the speed and angular.z = speed * tan(steer) / wheelbase.
"""
import math
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from robotek_common.vehicle import REAL_STEER_LIMIT, WHEELBASE

HELP = 'w/s: faster/slower   a/d: steer left/right   space: stop   q: quit'


class KeyboardDrive(Node):
    def __init__(self):
        super().__init__('keyboard_drive')
        self.pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.speed = 0.0
        self.steer = 0.0
        self.hold = bool(self.declare_parameter('hold', False).value)
        self.hold_speed = float(self.declare_parameter('hold_speed', 0.15).value)
        self.hold_timeout = float(self.declare_parameter('hold_timeout', 0.6).value)
        self.last_key = 0.0
        self.create_timer(0.05, self.tick)

    def key(self, ch):
        if self.hold:
            self.hold_key(ch)
            return
        if ch == 'w':
            self.speed = min(1.28, self.speed + 0.1)
        elif ch == 's':
            self.speed = max(-0.4, self.speed - 0.1)
        elif ch == 'a':
            self.steer = min(REAL_STEER_LIMIT, self.steer + 0.05)
        elif ch == 'd':
            self.steer = max(-REAL_STEER_LIMIT, self.steer - 0.05)
        elif ch == ' ':
            self.speed, self.steer = 0.0, 0.0
        print(f'\rspeed {self.speed:+.1f} m/s   steer {math.degrees(self.steer):+5.1f} deg   ', end='', flush=True)

    def hold_key(self, ch):
        now = self.get_clock().now().nanoseconds * 1e-9
        if ch in ('w', 's'):
            self.speed = self.hold_speed if ch == 'w' else -self.hold_speed
            self.last_key = now
        elif ch == 'a':
            self.steer = min(REAL_STEER_LIMIT, self.steer + 0.05)
        elif ch == 'd':
            self.steer = max(-REAL_STEER_LIMIT, self.steer - 0.05)
        elif ch == 'x':
            self.steer = 0.0
        elif ch == ' ':
            self.speed = 0.0
        print(f'\rspeed {self.speed:+.2f} m/s   steer {math.degrees(self.steer):+5.1f} deg   ', end='', flush=True)

    def tick(self):
        if self.hold and self.speed != 0.0 and \
                self.get_clock().now().nanoseconds * 1e-9 - self.last_key > self.hold_timeout:
            self.speed = 0.0                       # key released: stop
        cmd = Twist()
        cmd.linear.x = self.speed
        cmd.angular.z = self.speed * math.tan(self.steer) / WHEELBASE
        self.pub.publish(cmd)


def main():
    if not sys.stdin.isatty():
        raise SystemExit('keyboard_drive needs a terminal (run it from a terminal, not from a launch file)')
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = KeyboardDrive()
    print('hold: keep w (forward) or s (back) pressed to move; a/d steer, x centre, q quit' if node.hold else HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if ch == 'q':
                    break
                node.key(ch)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.speed = 0.0
        for _ in range(5):                         # make sure the car stops
            node.tick()
        node.destroy_node()
        rclpy.try_shutdown()
        print()


if __name__ == '__main__':
    main()
