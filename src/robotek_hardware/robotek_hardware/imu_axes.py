"""Find which board axis of the RRC Lite IMU is up, forward and left (run on the car, stack stopped).

    ros2 run robotek_hardware imu_axes                 # listens to /rrc/imu_raw

1. Leave the car still on the table for `rest_seconds` (gravity gives the up axis).
2. When it says PUSH, push the car forward by hand once, gently, and let go (the first
   acceleration burst gives the forward axis). The left axis follows from up x forward.
It prints the three parameters for robot.yaml (imu_adapter) and how far each direction is from the
nearest board axis (more than ~15 degrees: tilted board or a crooked push, repeat).
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from .mapping import estimate_axes


def forward_burst(samples, threshold=.4):
    """Mean of (a - rest) over the first acceleration burst: from the first sample above
    `threshold` until the acceleration flips sign (the stop)."""
    burst, direction = [], None
    for sample in samples:
        norm = math.sqrt(sum(v*v for v in sample))
        if direction is None:
            if norm < threshold:
                continue
            direction = [v/norm for v in sample]
        elif sum(v*d for v, d in zip(sample, direction)) < 0.:
            break
        burst.append(sample)
    if not burst:
        raise ValueError('no push seen: push harder or lower the threshold')
    return [sum(s[i] for s in burst)/len(burst) for i in range(3)]


class ImuAxes(Node):
    def __init__(self):
        super().__init__('imu_axes')
        self.rest_seconds = float(self.declare_parameter('rest_seconds', 3.).value)
        self.push_seconds = float(self.declare_parameter('push_seconds', 8.).value)
        topic = self.declare_parameter('topic', 'rrc/imu_raw').value
        self.rest, self.push, self.phase = [], [], 'rest'
        self.start = time.monotonic()
        self.get_logger().info(f'Leave the car still for {self.rest_seconds:.0f} s...')
        self.create_subscription(Imu, topic, self.on_imu, qos_profile_sensor_data)
        self.create_timer(.2, self.tick)
        self.done = False

    def on_imu(self, msg):
        a = msg.linear_acceleration
        if self.phase == 'rest':
            self.rest.append((a.x, a.y, a.z))
        elif self.phase == 'push':
            self.push.append((a.x, a.y, a.z))

    def tick(self):
        elapsed = time.monotonic() - self.start
        if self.phase == 'rest' and elapsed > self.rest_seconds:
            if len(self.rest) < 20:
                raise SystemExit('no IMU data: is the RRC Lite driver running?')
            self.rest_mean = [sum(s[i] for s in self.rest)/len(self.rest) for i in range(3)]
            self.phase, self.start = 'push', time.monotonic()
            self.get_logger().info(f'PUSH the car forward once now ({self.push_seconds:.0f} s window)')
        elif self.phase == 'push' and elapsed > self.push_seconds:
            self.phase, self.done = 'done', True
            deltas = [tuple(v - r for v, r in zip(s, self.rest_mean)) for s in self.push]
            try:
                push_mean = forward_burst(deltas)
                found = estimate_axes(self.rest_mean, [r + p for r, p in zip(self.rest_mean, push_mean)])
            except ValueError as error:
                raise SystemExit(str(error))
            print('imu_adapter:\n  ros__parameters:\n'
                  f"    forward_axis: '{found['forward']}'\n    left_axis: '{found['left']}'\n"
                  f"    up_axis: '{found['up']}'\n    measured: true")
            print('error_deg:', found['error_deg'])


def main(args=None):
    rclpy.init(args=args)
    node = ImuAxes()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.2)
    except (KeyboardInterrupt, SystemExit) as error:
        if isinstance(error, SystemExit) and error.code:
            print(error)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
