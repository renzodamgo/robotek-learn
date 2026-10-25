"""Simulation-only LiDAR model for real_car:=true: random ray dropouts like the LD19.

Gazebo's gpu_lidar only adds gaussian noise. On the car about 4.3 % of the rays inside
the +-110 deg field come back empty each revolution, at random places (measured 8 oct
2026 over 121 scans). This node takes the Gazebo scan
(/scan_gz) and publishes /scan with each ray set to inf with probability ``dropout``.
Never used on the real robot.
"""
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def drop_rays(ranges, dropout, rng):
    """Copy of ranges with each value replaced by inf with probability dropout."""
    return [math.inf if rng.random() < dropout else r for r in ranges]


class SimScanModel(Node):
    def __init__(self):
        super().__init__('sim_scan_model')
        self.dropout = float(self.declare_parameter('dropout', .043).value)
        seed = int(self.declare_parameter('seed', 0).value)
        self.rng = random.Random(seed or None)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.create_subscription(LaserScan, '/scan_gz', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        msg.ranges = drop_rays(msg.ranges, self.dropout, self.rng)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SimScanModel()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
