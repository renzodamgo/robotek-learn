#!/usr/bin/env python3
"""Measure the real wheel angle for each steering servo pulse, on the floor, with the gyroscope.

    # on the car, with car.launch.py running but base_adapter stopped (it would overwrite the servo and motor):
    python3 tools/steer_calibration.py 1300 1400 1500 1600 1700        # servo pulses (us) to measure

For each pulse the car drives forward at a fixed motor speed for a few seconds and the gyroscope gives the yaw rate
w. The board's speed control keeps the rear wheels at the same speed with any steering, so the speed v measured
once on a straight run toward a wall (LiDAR) holds for every pulse, and the bicycle model gives the wheel angle:
delta = atan(wheelbase * w / v) (+ = left). The pulse with delta = 0 is the real centre; the extreme pulses give the
largest angle on each side. Needs ~1.5 m free in front (speed run) and ~1 m around the car (each arc).

Safety: the motor stops if something is closer than STOP_M in front, at the end of each run and on Ctrl+C; the
board's motor watchdog stops it too if this script dies.
"""
import math
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rrc_lite_msgs.msg import MotorsState, MotorState, SetAckerServoState
from sensor_msgs.msg import Imu, LaserScan

WHEELBASE = 0.1174   # m (robot.yaml)
MOTOR_PORT = 4
MOTOR_SIGN = -1.0    # +rps drives the car backward (robot.yaml rrc_motor_sign)
RPS = 1.0            # the board's repeatable speed range
RUN_S = 3.0          # s per pulse; w is averaged after SETTLE_S
SETTLE_S = 1.0
STOP_M = 0.25


class SteerCalibration:
    def __init__(self, node):
        self.node = node
        self.servo = node.create_publisher(SetAckerServoState, '/rrc/acker_servo/set_state', 10)
        self.motor = node.create_publisher(MotorsState, '/rrc/set_motor', 10)
        self.gyro = None
        self.front = math.inf
        node.create_subscription(Imu, '/robotek/imu', self.on_imu, qos_profile_sensor_data)
        node.create_subscription(LaserScan, '/robotek/scan', self.on_scan, qos_profile_sensor_data)

    def on_imu(self, msg):
        self.gyro = msg.angular_velocity.z

    def on_scan(self, msg):
        near = sorted(r for i, r in enumerate(msg.ranges)
                      if abs(math.degrees(msg.angle_min + i*msg.angle_increment)) <= 5 and msg.range_min < r < msg.range_max)
        self.front = near[len(near)//2] if near else math.inf

    def spin(self, seconds, each=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if each:
                each()

    def set_servo(self, us):
        self.servo.publish(SetAckerServoState(duration=0.2, position=int(us)))

    def drive(self, rps):
        self.motor.publish(MotorsState(data=[MotorState(id=MOTOR_PORT, rps=float(MOTOR_SIGN*rps))]))

    def stop(self):
        for _ in range(3):
            self.drive(0.)
            self.spin(0.05)

    def bias(self):
        samples = []
        self.spin(0.5)
        self.spin(1.5, lambda: self.gyro is not None and samples.append(self.gyro))
        return sum(samples)/len(samples)

    def run(self, us, seconds, record):
        """Drive forward at RPS with the servo at us; record(t, gyro, front) after SETTLE_S. False if blocked."""
        self.set_servo(us)
        self.spin(0.4)
        start = time.monotonic()
        blocked = []

        def step():
            if self.front < STOP_M:
                blocked.append(True)
                return
            self.drive(RPS)
            t = time.monotonic() - start
            if t >= SETTLE_S and self.gyro is not None:
                record(t, self.gyro, self.front)

        end = start + seconds
        while time.monotonic() < end and not blocked:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            step()
        self.stop()
        return not blocked


def slope(points):
    n = len(points)
    mt = sum(p[0] for p in points)/n
    my = sum(p[1] for p in points)/n
    return sum((p[0]-mt)*(p[1]-my) for p in points)/sum((p[0]-mt)**2 for p in points)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    pulses = [float(a) for a in args]
    centre = float(next((a.split('=')[1] for a in sys.argv[1:] if a.startswith('--centre=')), 1500))
    speed = next((float(a.split('=')[1]) for a in sys.argv[1:] if a.startswith('--speed=')), None)
    rclpy.init()
    node = rclpy.create_node('steer_calibration')
    cal = SteerCalibration(node)
    try:
        cal.spin(1.0)
        if cal.gyro is None:
            sys.exit('No /robotek/imu: is car.launch.py running?')
        bias = cal.bias()
        print(f'gyro bias {math.degrees(bias):+.2f} deg/s')
        if speed is None:
            if cal.front < 1.2:
                sys.exit(f'Speed run needs >= 1.2 m free in front (now {cal.front:.2f} m), or pass --speed=<m/s>.')
            pts = []
            cal.run(centre, 2.5, lambda t, g, f: math.isfinite(f) and pts.append((t, f)))
            speed = -slope(pts)
            print(f'speed at {RPS} rps: {speed:.3f} m/s (LiDAR, servo {centre:.0f} us)')
            cal.spin(1.0)
        for us in pulses:
            rates = []
            ok = cal.run(us, RUN_S, lambda t, g, f: rates.append(g - bias))
            if not rates:
                print(f'{us:6.0f} us: no data{" (blocked in front)" if not ok else ""}')
                continue
            w = sum(rates)/len(rates)
            delta = math.degrees(math.atan(WHEELBASE*w/speed))
            radius = speed/w if abs(w) > 1e-3 else math.inf
            print(f'{us:6.0f} us: wheel {delta:+6.2f} deg ({"left" if delta > 0 else "right"}), '
                  f'yaw {math.degrees(w):+6.1f} deg/s, radius {radius:+.2f} m{"" if ok else " (stopped early)"}')
            cal.spin(0.8)
        cal.set_servo(centre)
    finally:
        cal.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
