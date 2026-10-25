"""Step 3: drive straight (step 2) and, meanwhile, turn every LiDAR scan into the four numbers of a Scene."""
import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float32MultiArray
from visualization_msgs.msg import MarkerArray

from robotek_common.vehicle import WHEELBASE
from robotek_course.minisim import Scene
from robotek_course.viz import scene_markers

from .heading import GyroHeading, heading_steer
from .perception import perceive

CONTROL_RATE = 50.0       # Hz, on the simulation clock
SPEED = 0.6               # m/s commanded
CALIBRATE_SECONDS = 0.6   # standing still after Start, measuring the gyroscope bias
STOP_FRONT = 0.5          # m: stop when the wall ahead is this near


class Drive(Node):
    def __init__(self):
        super().__init__('strategy')
        self.heading = GyroHeading()
        self.gyro = 0.0
        self.samples = []
        self.time = None
        self.stopped = False                 # latched: a noisy reading must not restart the car
        self.scene = Scene()                 # the latest Scene (nothing seen yet)
        self.stamp = None                    # time stamp of the last scan we used
        self.cmd = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.scene_pub = self.create_publisher(Float32MultiArray, '/scene', 10)      # left, right, front_axis, island_end
        self.markers_pub = self.create_publisher(MarkerArray, '/scene_markers', 10)  # the same, to see it in RViz
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Bool, 'start', self.on_start, 10)   # plain QoS: works with the referee and with the car's button
        self.create_timer(1.0 / CONTROL_RATE, self.control)

    def on_scan(self, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.stamp:
            return                           # the same scan can arrive more than once: use each time stamp once
        self.stamp = stamp
        scene = self.scene = perceive(msg, forward_yaw=0.0, sign=1)
        numbers = [scene.left, scene.right, scene.front_axis, scene.island_end]
        self.scene_pub.publish(Float32MultiArray(data=[float(n) for n in numbers]))
        self.markers_pub.publish(scene_markers(scene, msg.header.stamp, msg.header.frame_id))

    def on_imu(self, msg):
        self.gyro = msg.angular_velocity.z

    def on_start(self, msg):
        if msg.data and self.time is None:
            self.get_logger().info('Start')
            self.time = 0.0

    def control(self):
        dt = 1.0 / CONTROL_RATE
        speed, steer = 0.0, 0.0
        if self.time is not None:
            self.time += dt
            if self.time < CALIBRATE_SECONDS:
                self.samples.append(self.gyro)
            else:
                if self.samples:
                    self.heading.calibrate(self.samples)
                    self.samples = []
                yaw = self.heading.update(self.gyro, dt)
                self.stopped = self.stopped or self.scene.front_axis < STOP_FRONT
                if not self.stopped:
                    speed, steer = SPEED, heading_steer(0.0, yaw)
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
            node.cmd.publish(Twist())       # leave the car stopped
        rclpy.try_shutdown()
