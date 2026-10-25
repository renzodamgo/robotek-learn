"""Drive the car with the keyboard (step 0): w/s speed, a/d steering, space stops, q quits.

    ros2 run robotek_course keyboard_drive

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
        self.create_timer(0.05, self.tick)

    def key(self, ch):
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

    def tick(self):
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
    print(HELP)
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
        node.tick()
        node.destroy_node()
        rclpy.try_shutdown()
        print()


if __name__ == '__main__':
    main()
