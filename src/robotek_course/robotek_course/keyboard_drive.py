"""Drive the car with the keyboard (step 0): w/s speed, a/d steering, space stops, q quits.

    ros2 run robotek_course keyboard_drive
    ros2 run robotek_course keyboard_drive --ros-args -p hold:=true     # real car: move it to the start

hold:=true (to place the real car, with car.launch.py teleop:=true): the car moves only while w (forward) or s (back)
is held, at hold_speed m/s (0.2: lower did not move the car on the encoder motor), and stops hold_timeout s after the key is released (the terminal repeats a held key);
a/d steer, x centres the wheels. To move and turn at once (a terminal cannot tell two held keys apart), one key per
combination, like teleop_twist_keyboard:  u i o  = forward-left, forward, forward-right;  j k l = steer left, stop,
steer right;  m , .  = back-left, back, back-right (turning keys use hold_steer degrees of wheel angle).
Leaving (q or Ctrl+C) sends zero speed.

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
        self.hold_speed = float(self.declare_parameter('hold_speed', 0.2).value)  # 0.15 (0.72 rps) did not move the car
        self.hold_timeout = float(self.declare_parameter('hold_timeout', 0.6).value)
        self.hold_steer = math.radians(float(self.declare_parameter('hold_steer', 15.0).value))
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
        moves = {'w': (1, None), 's': (-1, None), 'i': (1, 0.), ',': (-1, 0.),
                 'u': (1, 1.), 'o': (1, -1.), 'm': (-1, 1.), '.': (-1, -1.)}
        if ch in moves:
            direction, turn = moves[ch]
            self.speed = direction * self.hold_speed
            if turn is not None:
                self.steer = max(-REAL_STEER_LIMIT, min(REAL_STEER_LIMIT, turn * self.hold_steer))
            self.last_key = now
        elif ch in ('a', 'j'):
            self.steer = min(REAL_STEER_LIMIT, self.steer + 0.05)
        elif ch in ('d', 'l'):
            self.steer = max(-REAL_STEER_LIMIT, self.steer - 0.05)
        elif ch == 'x':
            self.steer = 0.0
        elif ch in (' ', 'k'):
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
    print('hold a key to move, release to stop:\n'
          '   u i o    forward-left  forward  forward-right\n'
          '   j k l    steer left    stop     steer right\n'
          '   m , .    back-left     back     back-right\n'
          '   w/s forward/back straight-on, a/d steer, x centre, q quit' if node.hold else HELP)
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
