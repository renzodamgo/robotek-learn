"""Step 6: + following the line: the car drives one lap on its own and stops.

The car drives one lap on its own, following the planned line at 0.25 m/s, and stops. Watch:

  ros2 topic echo /obstacle/pose      # x, y, heading (track frame) and progress
  ros2 topic echo /obstacle/signs     # the seats remembered so far (from step 3)
  ros2 topic echo /obstacle/plan      # the planned offset 0, 0.5, 1 and 1.5 units ahead (from step 4)
"""
import json
import math

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String

from robotek_common.lidar_odometry import LidarOdometry
from robotek_common.vehicle import (DISTANCE_MODEL_ACCEL, OUTLINE_HALF_WIDTH, REAL_CAMERA_HFOV, REAL_CAMERA_PITCH,
                                    REAL_CAMERA_YAW, WHEELBASE)
from robotek_course.obstacle_vision import associate, color_blocks, scan_clusters

from . import lane
from .localizer import Localizer, direction_from_scan, scan_points

STEP = 6
if STEP >= 3:
    from .signs import SeatMemory
if STEP >= 4:
    from . import plan
if STEP >= 5:
    from . import corner
if STEP >= 6:
    from . import follow

CONTROL_RATE = 50.0       # Hz, on the simulation clock
CALIBRATE_SECONDS = 0.3   # standing still after Start, measuring the gyroscope bias
CLEARANCE = 0.18 + OUTLINE_HALF_WIDTH   # m from a sign to the car's centre line (step 4)
SPEED = 0.25              # m/s while following the line (step 6)
STEER_LIMIT = 0.307       # rad, the real car
LOOKAHEAD = 1.8           # m ahead a sign is looked for


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Drive(Node):
    def __init__(self):
        super().__init__('strategy')
        self.loc = Localizer()
        self.odometry = LidarOdometry(accel=DISTANCE_MODEL_ACCEL)
        self.memory = SeatMemory() if STEP >= 3 else None
        self.bridge = CvBridge()
        self.started = None
        self.sign = 0.0
        self.calib = []
        self.bias = 0.0
        self.yaw = self.gyro = 0.0
        self.imu_time = None
        self.yaw_zero = self.th0 = 0.0
        self.scan = None
        self.stamp = -math.inf
        self.used = -math.inf
        self.phi = 0.0
        self.knots = []
        self.observations, self.obs_stamp = [], -math.inf
        self.command = 0.0                   # the speed commanded (by the keyboard until step 6)
        self.cmd = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.pose_pub = self.create_publisher(Float32MultiArray, '/obstacle/pose', 10)
        self.signs_pub = self.create_publisher(String, '/obstacle/signs', 10)
        self.plan_pub = self.create_publisher(Float32MultiArray, '/obstacle/plan', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image', self.on_image, qos_profile_sensor_data)
        if STEP < 6:
            self.create_subscription(Twist, '/cmd_vel_raw', self.on_command, 10)   # the keyboard's commands
        self.create_subscription(Bool, 'start', self.on_start, 10)
        self.create_timer(1.0 / CONTROL_RATE, self.control)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ sensors
    def on_scan(self, msg):
        stamp = stamp_of(msg)
        if stamp > self.stamp:
            self.odometry.scan(msg, self.yaw)
            self.stamp = stamp
        self.scan = msg

    def on_imu(self, msg):
        stamp = stamp_of(msg)
        if self.imu_time is not None and 0.0 < stamp - self.imu_time < 0.2:
            self.yaw += (msg.angular_velocity.z - self.bias) * (stamp - self.imu_time)
        self.imu_time = stamp
        self.gyro = msg.angular_velocity.z

    def on_image(self, msg):
        if self.scan is None or abs(stamp_of(msg) - self.stamp) > 0.15:
            return
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        blocks = color_blocks(bgr, hfov=REAL_CAMERA_HFOV, pitch=REAL_CAMERA_PITCH, yaw=REAL_CAMERA_YAW)
        self.observations, self.obs_stamp = associate(blocks, scan_clusters(self.scan)), stamp_of(msg)

    def on_command(self, msg):
        self.command = msg.linear.x

    def on_start(self, msg):
        if msg.data and self.started is None:
            self.get_logger().info('Start')
            self.started = self.now()

    # ------------------------------------------------------------------ control
    def control(self):
        self.odometry.command(self.command, 1.0 / CONTROL_RATE)
        distance = self.odometry.read()
        if self.started is None or self.scan is None:
            return
        if self.now() - self.started < CALIBRATE_SECONDS:
            self.calib.append(self.gyro)
            return
        if not self.loc.ready:
            self.bias = sum(self.calib) / max(len(self.calib), 1)
            sign, margin = direction_from_scan(self.scan)
            if margin < 0.03 or not sign:
                return
            self.sign, self.yaw_zero = sign, self.yaw
            self.loc.initialize(*scan_points(self.scan, sign), 0.0)
            self.th0 = self.loc.th
            self.phi = float(lane.track_s(self.loc.x, self.loc.y))
            self.get_logger().info('Direction %s' % ('ccw' if sign > 0 else 'cw'))
            return
        # Pose: predict with distance + gyro, correct with the walls on every new scan.
        self.loc.predict(distance if self.command >= 0 else -distance, self.sign * lane.wrap(self.yaw - self.yaw_zero) + self.th0)
        new_scan = self.stamp > self.used
        if new_scan:
            self.used = self.stamp
            bx, by = scan_points(self.scan, self.sign)
            if bx.size >= 20:
                before = self.loc.th
                self.loc.correct(bx, by, 0.5)
                self.th0 += lane.wrap(self.loc.th - before)
        pose = (self.loc.x, self.loc.y, self.loc.th)
        self.phi = float(lane.unwrap_s(self.phi, lane.track_s(pose[0], pose[1])))
        self.pose_pub.publish(Float32MultiArray(data=[float(v) for v in (*pose, self.phi)]))
        if STEP >= 3 and new_scan:
            self.memory.remember(scan_clusters(self.scan), self.observations, self.obs_stamp, self.stamp,
                                 pose, self.phi, self.sign, LOOKAHEAD)
            seats = {str(k / 2.0): [lateral, self.memory.colour(k), hits]
                     for k, (lateral, _, hits) in sorted(self.memory.seats.items())}
            self.signs_pub.publish(String(data=json.dumps(seats)))
        if STEP >= 4:
            signs = plan.with_offsets(self.memory.ahead(self.phi, 1.5, plan.HORIZON + 1.0), CLEARANCE)
            knots = sorted(plan.sign_knots(signs, self.phi))
            if STEP >= 5:
                knots = corner.merge(knots, signs, self.phi, plan.HORIZON)
            self.knots = knots
            ahead = [plan.offset_at(knots, self.phi + d) for d in (0.0, 0.5, 1.0, 1.5)]
            self.plan_pub.publish(Float32MultiArray(data=[float(v) for v in ahead]))
        if STEP >= 6:
            # Follow the line for one lap and stop (the mission, step 7, does the rest).
            if self.phi >= 8.7:
                speed, steer = 0.0, 0.0
            else:
                steer = self.sign * follow.steer(pose, self.phi, (self.gyro - self.bias) * 1.0,
                                                 lambda s: plan.offset_at(self.knots, s), 1.5, 0.2, STEER_LIMIT)
                speed = SPEED
            self.command = speed
            cmd = Twist()
            cmd.linear.x = float(speed)
            cmd.angular.z = float(speed * math.tan(steer) / WHEELBASE)
            self.cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Drive()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        if rclpy.ok():
            node.cmd.publish(Twist())
            rclpy.shutdown()
