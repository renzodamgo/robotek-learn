"""VL53L0X time-of-flight sensor on the Pi's I2C bus -> /robotek/rear_range (sensor_msgs/Range, metres) and
/robotek/rear_range_mm (std_msgs/Int32, millimetres; -1 when nothing is in range).

Wiring: VIN -> 3.3 V (pin 1), SDA -> GPIO 2 (pin 3), SCL -> GPIO 3 (pin 5), GND (pin 9). Address 0x29 on bus 1.

The sensor is read without ST's API: write 0x01 to SYSRANGE_START, wait for bit 0 of RESULT_RANGE_STATUS, read 12
bytes; bytes 10-11 are the distance in mm and bits 3-6 of byte 0 the range status (11 = valid). It uses the
sensor's default settings (about 30 ms per reading, up to ~1.2 m indoors). A VL53L1X is not supported: the node
reads the model id (0xEE = VL53L0X) and says so. Uses lgpio for I2C, like gpio_button; on a Mac or without the
sensor it logs it and keeps retrying, it never crashes the launch."""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Int32

try:
    import lgpio
except ImportError:  # only on the Raspberry Pi (python3-lgpio)
    lgpio = None

SYSRANGE_START = 0x00
SYSTEM_INTERRUPT_CLEAR = 0x0B
RESULT_RANGE_STATUS = 0x14
MODEL_ID = 0xC0
VL53L0X_ID = 0xEE
STATUS_VALID = 11
NO_TARGET_MM = 8000      # the sensor reports 8190/8191 when nothing reflects


def decode(block):
    """12 bytes from RESULT_RANGE_STATUS -> (distance in mm, range status)."""
    return (block[10] << 8) | block[11], (block[0] & 0x78) >> 3


class RearRange(Node):
    def __init__(self):
        super().__init__('rear_range')
        p = {k: self.declare_parameter(k, v).value for k, v in dict(
            bus=1, address=0x29, poll_s=.01, min_m=.03, max_m=1.2, fov_rad=.44, frame_id='rear_range_link',
            topic='/robotek/rear_range', topic_mm='/robotek/rear_range_mm').items()}
        self.bus, self.address = int(p['bus']), int(p['address'])
        self.min_m, self.max_m, self.fov = float(p['min_m']), float(p['max_m']), float(p['fov_rad'])
        self.frame_id = str(p['frame_id'])
        self.pub = self.create_publisher(Range, str(p['topic']), 10)
        self.pub_mm = self.create_publisher(Int32, str(p['topic_mm']), 10)
        self.i2c = None
        self.measuring = False
        self.waited = 0
        self.readings = 0
        if lgpio is None:
            self.get_logger().error('lgpio missing (sudo apt install python3-lgpio): rear range disabled.')
            return
        if not self.open_i2c():
            self.create_timer(5., self.retry_i2c)
        self.create_timer(float(p['poll_s']), self.poll)

    def open_i2c(self):
        handle = None
        try:
            handle = lgpio.i2c_open(self.bus, self.address)
            model = lgpio.i2c_read_byte_data(handle, MODEL_ID)
        except Exception as error:  # lgpio.error: no bus, permission denied, no sensor answering
            self.close(handle)
            self.get_logger().warn(
                f'No sensor on I2C bus {self.bus} address 0x{self.address:02x} ({error}): retrying every 5 s.',
                throttle_duration_sec=60.)
            return False
        if model != VL53L0X_ID:
            self.close(handle)
            self.get_logger().error(
                f'I2C 0x{self.address:02x} answers model id 0x{model:02x}, not 0x{VL53L0X_ID:02x} (VL53L0X). '
                'A VL53L1X is not supported by this node.', throttle_duration_sec=60.)
            return False
        self.i2c = handle
        self.get_logger().info(f'VL53L0X on I2C bus {self.bus} address 0x{self.address:02x}.')
        return True

    @staticmethod
    def close(handle):
        if handle is not None:
            try:
                lgpio.i2c_close(handle)
            except Exception:  # noqa: BLE001
                pass

    def retry_i2c(self):
        if self.i2c is None:
            self.open_i2c()

    def poll(self):
        if self.i2c is None:
            return
        try:
            if not self.measuring:
                lgpio.i2c_write_byte_data(self.i2c, SYSRANGE_START, 0x01)
                self.measuring, self.waited = True, 0
                return
            if not lgpio.i2c_read_byte_data(self.i2c, RESULT_RANGE_STATUS) & 0x01:
                self.waited += 1
                if self.waited > 50:    # 0.5 s without a result: start again
                    self.measuring = False
                    self.get_logger().warn('VL53L0X did not answer in time.', throttle_duration_sec=10.)
                return
            _, data = lgpio.i2c_read_i2c_block_data(self.i2c, RESULT_RANGE_STATUS, 12)
            lgpio.i2c_write_byte_data(self.i2c, SYSTEM_INTERRUPT_CLEAR, 0x01)
        except Exception as error:  # noqa: BLE001
            self.measuring = False
            self.get_logger().warn(f'I2C read failed ({error}).', throttle_duration_sec=10.)
            return
        self.measuring = False
        self.publish(*decode(bytes(data)))

    def publish(self, mm, status):
        valid = status == STATUS_VALID and 0 < mm < NO_TARGET_MM
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = self.fov
        msg.min_range, msg.max_range = self.min_m, self.max_m
        msg.range = mm/1000. if valid else float('inf')
        self.pub.publish(msg)
        self.pub_mm.publish(Int32(data=mm if valid else -1))
        self.readings += 1
        if self.readings == 1:
            self.get_logger().info(f'First reading: {mm} mm (status {status}).')

    def destroy_node(self):
        self.close(self.i2c)
        self.i2c = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RearRange()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
