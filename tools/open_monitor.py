#!/usr/bin/env python3
"""Monitor of the Open on the real car: twice a second, what the strategy sees and commands.

    source install/setup.bash
    python3 tools/open_monitor.py [seconds] [--log FILE]     # in a second terminal, while open_challenge.launch.py runs

Columns: time | state (/robotek/state) | camera blue/orange line length (px, the direction vote) | lines crossed
blue/orange (LineCounter, every frame) | LiDAR left, right, front (m, /robotek/scan) | gyro z (rad/s, /robotek/imu) |
command: speed (m/s) and steering (deg, + = left, from /cmd_vel_raw) | drive motor rps (/rrc/set_motor).
3.5 s after Start it prints the gyro bias at rest before Start and in the strategy's calibration window
(0.5-2.0 s after Start: tools/patches/my_open-car-calibration.patch), to check the calibration.

The camera ranges and cuts are the car's (the same defaults as car.launch.py). Nothing here commands the car.
"""
import math
import os
import statistics
import sys
import time

for key, value in (('ROBOTEK_CAMERA_BOTTOM_CROP', '0.12'), ('ROBOTEK_CAMERA_WALL_MASK', '1'),
                   ('ROBOTEK_CAMERA_BLUE', '95,135,15,35,125'), ('ROBOTEK_CAMERA_ORANGE', '155,10,25,35,255')):
    os.environ.setdefault(key, value)

import rclpy  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Image, Imu, LaserScan  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402

from robotek_common.vehicle import WHEELBASE  # noqa: E402
from robotek_course.vision import LineCounter, line_lengths  # noqa: E402

try:
    from rrc_lite_msgs.msg import MotorsState
except ImportError:
    MotorsState = None


def window(scan, centre, half):
    """Median range within +-half (rad) of centre."""
    values = []
    for i, r in enumerate(scan.ranges):
        a = scan.angle_min + i * scan.angle_increment
        if math.isfinite(r) and r > 0 and abs(math.atan2(math.sin(a - centre), math.cos(a - centre))) <= half:
            values.append(r)
    return statistics.median(values) if values else math.inf


def main():
    args = [a for a in sys.argv[1:]]
    log = None
    if '--log' in args:
        log = open(args[args.index('--log') + 1], 'w')
        del args[args.index('--log'):args.index('--log') + 2]
    seconds = float(args[0]) if args else 600.0

    def out(text):
        print(text, flush=True)
        if log:
            log.write(text + '\n')
            log.flush()

    rclpy.init()
    node = rclpy.create_node('open_monitor')
    bridge = CvBridge()
    counter = LineCounter()
    s = {'state': '-', 'cam': (0, 0), 'lines': (0, 0), 'scan': (math.inf,) * 3, 'gyro': 0.0, 'cmd': (0.0, 0.0),
         'rps': 0.0, 'cam_t': 0.0, 'start': None}
    gyro_log = []

    def on_scan(m):
        s['scan'] = (window(m, math.pi / 2, 0.44), window(m, -math.pi / 2, 0.44), window(m, 0.0, math.radians(3)))

    def on_imu(m):
        s['gyro'] = m.angular_velocity.z
        gyro_log.append((time.monotonic(), m.angular_velocity.z))

    def on_image(m):
        frame = bridge.imgmsg_to_cv2(m, 'bgr8')
        s['lines'] = counter.update(frame)
        if time.monotonic() - s['cam_t'] > 0.45:
            s['cam_t'] = time.monotonic()
            s['cam'] = tuple(round(x) for x in line_lengths(frame))

    def on_cmd(m):
        v, w = m.linear.x, m.angular.z
        s['cmd'] = (v, math.degrees(math.atan(w * WHEELBASE / v)) if abs(v) > 1e-3 else 0.0)

    node.create_subscription(LaserScan, '/robotek/scan', on_scan, qos_profile_sensor_data)
    node.create_subscription(Imu, '/robotek/imu', on_imu, qos_profile_sensor_data)
    node.create_subscription(Image, '/usb_cam/image_raw', on_image, qos_profile_sensor_data)
    node.create_subscription(String, '/robotek/state', lambda m: s.__setitem__('state', m.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Twist, '/cmd_vel_raw', on_cmd, 10)
    node.create_subscription(Bool, '/start', lambda m: s.__setitem__('start', time.monotonic())
                             if m.data and s['start'] is None else None, 10)
    if MotorsState is not None:
        node.create_subscription(MotorsState, '/rrc/set_motor',
                                 lambda m: s.__setitem__('rps', m.data[0].rps if m.data else 0.0), 10)

    out('t | state | cam blue/orange px | lines crossed blue/orange | L R F (m) | gyro z | cmd v (m/s) steer (deg) | rps')
    t0 = nxt = time.monotonic()
    reported = False
    try:
        while time.monotonic() - t0 < seconds:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if s['start'] and not reported and now - s['start'] > 3.5:
                reported = True
                s0 = s['start']

                def bias(lo, hi):
                    v = [g for t, g in gyro_log if s0 + lo <= t <= s0 + hi]
                    return f'{statistics.mean(v):+.4f} (n {len(v)})' if v else '-'
                out(f'GYRO BIAS at rest before Start {bias(-5.0, -0.5)} | calibration window 0.5-2.0 s {bias(0.5, 2.0)}')
            if now >= nxt:
                nxt += 0.5
                left, right, front = s['scan']
                v, steer = s['cmd']
                out(f"{now - t0:6.1f} | {s['state']:<16} | {s['cam'][0]:3d}/{s['cam'][1]:3d} | "
                    f"{s['lines'][0]:2d}/{s['lines'][1]:2d} | {left:4.2f} {right:4.2f} {front:4.2f} | "
                    f"{s['gyro']:+.2f} | {v:+.2f} {steer:+5.1f} | {s['rps']:+.2f}")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
