"""RRC Lite IMU (board axes) -> car frame: x forward, y left, z up.

The strategies integrate the gyro z as the yaw rate, so on the car /imu must already be in the car
frame. With the Raspberry Pi and the RRC Lite standing on their long edge (new build, 21 oct 2026)
the board axes no longer line up with the car: name, per car axis, the board axis that points that
way (e.g. forward_axis '+y'). Measure them with `ros2 run robotek_hardware imu_axes`.
Orientation stays zero (the board does not give one); covariances are rotated with the axes.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from .mapping import axes_matrix, remap_covariance, remap_vector


class ImuAdapter(Node):
    def __init__(self):
        super().__init__('imu_adapter')
        forward = self.declare_parameter('forward_axis', '+y').value
        left = self.declare_parameter('left_axis', '-x').value
        up = self.declare_parameter('up_axis', '+z').value
        self.frame = self.declare_parameter('frame_id', 'imu_link').value
        measured = self.declare_parameter('measured', False).value
        self.matrix = axes_matrix(forward, left, up)
        if not measured:
            self.get_logger().warn(
                f'IMU axes forward={forward} left={left} up={up} are the old flat build, not measured '
                'for the vertical board: run imu_axes and set measured: true in robot.yaml')
        else:
            self.get_logger().info(f'IMU axes forward={forward} left={left} up={up}')
        self.pub = self.create_publisher(Imu, 'imu_out', qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu_in', self.on_imu, qos_profile_sensor_data)

    def on_imu(self, msg):
        out = Imu()
        out.header = msg.header
        out.header.frame_id = self.frame
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        gyro = remap_vector((msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z), self.matrix)
        accel = remap_vector((msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z),
                             self.matrix)
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = gyro
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = accel
        out.angular_velocity_covariance = remap_covariance(msg.angular_velocity_covariance, self.matrix)
        out.linear_acceleration_covariance = remap_covariance(msg.linear_acceleration_covariance, self.matrix)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
