"""Lidar driver scan -> /robotek/scan with 0 rad straight ahead (CCW positive).

The STL-19P driver's zero points backwards (team repo: front 180 deg, left 240,
right 120), so the default offset is pi. Only the angles change; the ranges and
timing are untouched. `reverse` flips a clockwise-ordered driver.

`fov_deg` and `min_valid_range` (0 = off) turn into inf what the car cannot really see:
on the real car the LD19 only sees +-110 deg; behind, its rays hit the car itself at
2-8 cm (measured 8 oct 2026). The simulation with real_car:=true has the same field.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .mapping import mask_scan, rotate_scan_angles


class ScanAdapter(Node):
    def __init__(self):
        super().__init__('scan_adapter')
        self.offset = self.declare_parameter('angle_offset', math.pi).value
        self.reverse = self.declare_parameter('reverse', False).value
        self.frame = self.declare_parameter('frame_id', 'lidar_link').value
        self.fov_deg = float(self.declare_parameter('fov_deg', 0.).value)
        self.min_valid = float(self.declare_parameter('min_valid_range', 0.).value)
        self.pub = self.create_publisher(LaserScan, 'scan_out', qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan_in', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        out = LaserScan()
        out.header = msg.header
        out.header.frame_id = self.frame
        out.time_increment, out.scan_time = msg.time_increment, msg.scan_time
        out.range_min, out.range_max = msg.range_min, msg.range_max
        ranges, intensities = list(msg.ranges), list(msg.intensities)
        angle_min, angle_max, increment = msg.angle_min, msg.angle_max, msg.angle_increment
        if self.reverse:
            ranges.reverse()
            intensities.reverse()
            angle_min, angle_max = -angle_max, -angle_min
        out.angle_min, out.angle_max = rotate_scan_angles(angle_min, angle_max, self.offset)
        out.angle_increment = abs(increment)
        if self.fov_deg > 0. or self.min_valid > 0.:
            ranges = mask_scan(ranges, out.angle_min, out.angle_increment, self.fov_deg, self.min_valid)
        out.ranges, out.intensities = ranges, intensities
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
