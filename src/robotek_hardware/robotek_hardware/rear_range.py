"""VL53L0X time-of-flight sensor on the Pi's I2C bus -> /robotek/rear_range (sensor_msgs/Range, metres) and
/robotek/rear_range_mm (std_msgs/Int32, millimetres; -1 when nothing is in range).

Wiring: VIN -> 3.3 V (pin 1), SDA -> GPIO 2 (pin 3), SCL -> GPIO 3 (pin 5), GND (pin 9). Address 0x29 on bus 1.

The sensor is driven without ST's API, with the register sequence of Pololu's VL53L0X library (Arduino, MIT): on
opening, the data init, the reference SPAD setup, ST's default tuning settings, the timing budget and the VHV and
phase reference calibrations; then continuous back-to-back ranging (about 33 ms per reading, up to ~1.2 m indoors).
Each result: wait for RESULT_INTERRUPT_STATUS, read 12 bytes from RESULT_RANGE_STATUS (bytes 10-11 = mm, bits 3-6
of byte 0 = range status, 11 = valid) and clear the interrupt. Without that init (factory registers only) the car's
sensor read 218 mm for a book at 100 mm (9 nov 2026).

Even initialised, the car's sensor reads long, and very close it barely changes. Raw readings against a wall
(9 nov 2026; ruler from the wall to the sensor's face unless noted): 10 mm -> 70, 20 -> 75, 30 -> 77, 40 -> 83,
50 -> 96, 85 -> 138 (LiDAR), 170 -> 231, ~420 -> ~476 (photo). The node maps raw to mm by linear interpolation in
that table (table_raw_mm, table_mm); past the last point it keeps slope 1 (raw ~ mm + 56), below the first it
follows the first segment, never under 0. Between 1 and 4 cm the raw reading moves only 2-6 mm per cm against its
+-3 mm noise, so there it is good to about +-1 cm; from 5 cm on to about +-5 mm. correct:=false publishes the raw
reading. (A book at 100 and 200 mm had given 188 and 332: not repeated against the wall, not used.)
A VL53L1X is not supported: the node reads the model id
(0xEE = VL53L0X) and says so. Uses lgpio for I2C, like gpio_button; on a Mac or without the
sensor it logs it and keeps retrying, it never crashes the launch."""
import time

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
SYSTEM_SEQUENCE_CONFIG = 0x01
SYSTEM_INTERRUPT_CLEAR = 0x0B
RESULT_INTERRUPT_STATUS = 0x13
RESULT_RANGE_STATUS = 0x14
MODEL_ID = 0xC0
VL53L0X_ID = 0xEE
STATUS_VALID = 11
NO_TARGET_MM = 8000      # the sensor reports 8190/8191 when nothing reflects


# ST's default tuning settings (VL53L0X_DefaultTuningSettings), as (register, value) writes in order.
TUNING = (
    (0xFF, 0x01), (0x00, 0x00), (0xFF, 0x00), (0x09, 0x00), (0x10, 0x00), (0x11, 0x00), (0x24, 0x01), (0x25, 0xFF),
    (0x75, 0x00), (0xFF, 0x01), (0x4E, 0x2C), (0x48, 0x00), (0x30, 0x20), (0xFF, 0x00), (0x30, 0x09), (0x54, 0x00),
    (0x31, 0x04), (0x32, 0x03), (0x40, 0x83), (0x46, 0x25), (0x60, 0x00), (0x27, 0x00), (0x50, 0x06), (0x51, 0x00),
    (0x52, 0x96), (0x56, 0x08), (0x57, 0x30), (0x61, 0x00), (0x62, 0x00), (0x64, 0x00), (0x65, 0x00), (0x66, 0xA0),
    (0xFF, 0x01), (0x22, 0x32), (0x47, 0x14), (0x49, 0xFF), (0x4A, 0x00), (0xFF, 0x00), (0x7A, 0x0A), (0x7B, 0x00),
    (0x78, 0x21), (0xFF, 0x01), (0x23, 0x34), (0x42, 0x00), (0x44, 0xFF), (0x45, 0x26), (0x46, 0x05), (0x40, 0x40),
    (0x0E, 0x06), (0x20, 0x1A), (0x43, 0x40), (0xFF, 0x00), (0x34, 0x03), (0x35, 0x44), (0xFF, 0x01), (0x31, 0x04),
    (0x4B, 0x09), (0x4C, 0x05), (0x4D, 0x04), (0xFF, 0x00), (0x44, 0x00), (0x45, 0x20), (0x47, 0x08), (0x48, 0x28),
    (0x67, 0x00), (0x70, 0x04), (0x71, 0x01), (0x72, 0xFE), (0x76, 0x00), (0x77, 0x00), (0xFF, 0x01), (0x0D, 0x01),
    (0xFF, 0x00), (0x80, 0x01), (0x01, 0xF8), (0xFF, 0x01), (0x8E, 0x01), (0x00, 0x01), (0xFF, 0x00), (0x80, 0x00))


def decode(block):
    """12 bytes from RESULT_RANGE_STATUS -> (distance in mm, range status)."""
    return (block[10] << 8) | block[11], (block[0] & 0x78) >> 3


def corrected(raw, table):
    """Raw mm -> real mm by linear interpolation in table [(raw, real), ...] (raw increasing); past the last point
    slope 1, below the first the first segment's slope; never under 0."""
    if raw >= table[-1][0]:
        return max(0, round(table[-1][1] + raw - table[-1][0]))
    i = 1 if raw < table[0][0] else next(k for k in range(1, len(table)) if raw < table[k][0])
    (r0, m0), (r1, m1) = table[i-1], table[i]
    return max(0, round(m0 + (raw-r0)*(m1-m0)/(r1-r0)))


def decode_timeout(reg):
    return ((reg & 0xFF) << (reg >> 8)) + 1


def encode_timeout(mclks):
    if mclks <= 0:
        return 0
    lsb, msb = mclks-1, 0
    while lsb > 0xFF:
        lsb, msb = lsb >> 1, msb+1
    return (msb << 8) | lsb


def macro_period_ns(vcsel_pclks):
    return (2304*vcsel_pclks*1655 + 500)//1000


def mclks_to_us(mclks, vcsel_pclks):
    ns = macro_period_ns(vcsel_pclks)
    return (mclks*ns + ns//2)//1000


def us_to_mclks(us, vcsel_pclks):
    ns = macro_period_ns(vcsel_pclks)
    return (us*1000 + ns//2)//ns


class Vl53l0x:
    """Register access and Pololu's init sequence over an lgpio I2C handle."""

    def __init__(self, handle, timeout_s=.5):
        self.h, self.timeout = handle, timeout_s
        self.stop_variable = 0

    def read(self, reg):
        return lgpio.i2c_read_byte_data(self.h, reg)

    def write(self, reg, value):
        lgpio.i2c_write_byte_data(self.h, reg, value & 0xFF)

    def read16(self, reg):
        _, data = lgpio.i2c_read_i2c_block_data(self.h, reg, 2)
        return (data[0] << 8) | data[1]

    def write16(self, reg, value):
        lgpio.i2c_write_i2c_block_data(self.h, reg, bytes(((value >> 8) & 0xFF, value & 0xFF)))

    def wait(self, ready, what):
        end = time.monotonic()+self.timeout
        while not ready():
            if time.monotonic() > end:
                raise TimeoutError(f'VL53L0X {what} timed out')
            time.sleep(.001)

    def init(self):
        # a sensor left ranging (by an earlier run, or after a reset that kept it in back-to-back mode) does not
        # finish the reference calibration: stop it first (9 nov 2026)
        self.write(SYSRANGE_START, 0x01)
        time.sleep(.05)
        self.write(SYSTEM_INTERRUPT_CLEAR, 0x01)
        self.write(0x89, self.read(0x89) | 0x01)    # I/O at 2.8 V (the boards have a 2.8 V regulator)
        self.write(0x88, 0x00)                      # I2C standard mode
        self.write(0x80, 0x01), self.write(0xFF, 0x01), self.write(0x00, 0x00)
        self.stop_variable = self.read(0x91)
        self.write(0x00, 0x01), self.write(0xFF, 0x00), self.write(0x80, 0x00)
        self.write(0x60, self.read(0x60) | 0x12)    # no MSRC / pre-range signal rate limit checks
        self.write16(0x44, int(.25*(1 << 7)))       # final range signal rate limit 0.25 MCPS
        self.write(SYSTEM_SEQUENCE_CONFIG, 0xFF)
        self.reference_spads()
        for reg, value in TUNING:
            self.write(reg, value)
        self.write(0x0A, 0x04)                      # interrupt on new sample ready
        self.write(0x84, self.read(0x84) & ~0x10)   # active low
        self.write(SYSTEM_INTERRUPT_CLEAR, 0x01)
        budget = self.timing_budget_us()
        self.write(SYSTEM_SEQUENCE_CONFIG, 0xE8)    # no MSRC and TCC
        self.set_timing_budget_us(budget)
        self.write(SYSTEM_SEQUENCE_CONFIG, 0x01)
        self.reference_calibration(0x40)            # VHV
        self.write(SYSTEM_SEQUENCE_CONFIG, 0x02)
        self.reference_calibration(0x00)            # phase
        self.write(SYSTEM_SEQUENCE_CONFIG, 0xE8)

    def reference_spads(self):
        self.write(0x80, 0x01), self.write(0xFF, 0x01), self.write(0x00, 0x00)
        self.write(0xFF, 0x06), self.write(0x83, self.read(0x83) | 0x04)
        self.write(0xFF, 0x07), self.write(0x81, 0x01), self.write(0x80, 0x01)
        self.write(0x94, 0x6B), self.write(0x83, 0x00)
        self.wait(lambda: self.read(0x83) != 0x00, 'SPAD info')
        self.write(0x83, 0x01)
        info = self.read(0x92)
        count, aperture = info & 0x7F, bool(info >> 7 & 0x01)
        self.write(0x81, 0x00), self.write(0xFF, 0x06), self.write(0x83, self.read(0x83) & ~0x04)
        self.write(0xFF, 0x01), self.write(0x00, 0x01), self.write(0xFF, 0x00), self.write(0x80, 0x00)
        _, spad_map = lgpio.i2c_read_i2c_block_data(self.h, 0xB0, 6)
        spad_map = bytearray(spad_map)
        self.write(0xFF, 0x01), self.write(0x4F, 0x00), self.write(0x4E, 0x2C), self.write(0xFF, 0x00)
        self.write(0xB6, 0xB4)
        first, enabled = (12 if aperture else 0), 0
        for i in range(48):
            if i < first or enabled == count:
                spad_map[i//8] &= ~(1 << (i % 8)) & 0xFF
            elif spad_map[i//8] >> (i % 8) & 0x01:
                enabled += 1
        lgpio.i2c_write_i2c_block_data(self.h, 0xB0, bytes(spad_map))

    def steps(self):
        config = self.read(SYSTEM_SEQUENCE_CONFIG)
        tcc, dss, msrc, pre, final = (bool(config >> b & 1) for b in (4, 3, 2, 6, 7))
        pre_pclks = (self.read(0x50)+1) << 1
        msrc_us = mclks_to_us(self.read(0x46)+1, pre_pclks)
        pre_mclks = decode_timeout(self.read16(0x51))
        final_pclks = (self.read(0x70)+1) << 1
        final_mclks = decode_timeout(self.read16(0x71)) - (pre_mclks if pre else 0)
        return dict(tcc=tcc, dss=dss, msrc=msrc, pre=pre, final=final, msrc_us=msrc_us, pre_mclks=pre_mclks,
                    pre_us=mclks_to_us(pre_mclks, pre_pclks), final_pclks=final_pclks,
                    final_us=mclks_to_us(final_mclks, final_pclks))

    @staticmethod
    def used_us(s):
        used = 1910+960
        if s['tcc']:
            used += s['msrc_us']+590
        if s['dss']:
            used += 2*(s['msrc_us']+690)
        elif s['msrc']:
            used += s['msrc_us']+660
        if s['pre']:
            used += s['pre_us']+660
        return used

    def timing_budget_us(self):
        s = self.steps()
        return self.used_us(s) + (s['final_us']+550 if s['final'] else 0)

    def set_timing_budget_us(self, budget):
        s = self.steps()
        if not s['final']:
            return
        used = self.used_us(s)+550
        if used > budget:
            raise ValueError(f'timing budget {budget} us below the {used} us the steps need')
        mclks = us_to_mclks(budget-used, s['final_pclks']) + (s['pre_mclks'] if s['pre'] else 0)
        self.write16(0x71, encode_timeout(mclks))

    def reference_calibration(self, vhv_init):
        self.write(SYSRANGE_START, 0x01 | vhv_init)
        self.wait(lambda: self.read(RESULT_INTERRUPT_STATUS) & 0x07, 'reference calibration')
        self.write(SYSTEM_INTERRUPT_CLEAR, 0x01)
        self.write(SYSRANGE_START, 0x00)

    def start_continuous(self):
        self.write(0x80, 0x01), self.write(0xFF, 0x01), self.write(0x00, 0x00)
        self.write(0x91, self.stop_variable)
        self.write(0x00, 0x01), self.write(0xFF, 0x00), self.write(0x80, 0x00)
        self.write(SYSRANGE_START, 0x02)            # back-to-back

    def ready(self):
        return bool(self.read(RESULT_INTERRUPT_STATUS) & 0x07)

    def result(self):
        _, data = lgpio.i2c_read_i2c_block_data(self.h, RESULT_RANGE_STATUS, 12)
        self.write(SYSTEM_INTERRUPT_CLEAR, 0x01)
        return decode(bytes(data))

    def stop(self):
        self.write(0x80, 0x01), self.write(0xFF, 0x01), self.write(0x00, 0x00)
        self.write(SYSRANGE_START, 0x01)
        self.write(0x91, 0x00)
        self.write(0x00, 0x01), self.write(0xFF, 0x00), self.write(0x80, 0x00)


class RearRange(Node):
    def __init__(self):
        super().__init__('rear_range')
        p = {k: self.declare_parameter(k, v).value for k, v in dict(
            bus=1, address=0x29, poll_s=.01, correct=True, table_raw_mm=[70., 75., 77., 83., 96., 138., 231., 476.],
            table_mm=[10., 20., 30., 40., 50., 85., 170., 420.], min_m=.03, max_m=1.2, fov_rad=.44, frame_id='rear_range_link',
            topic='/robotek/rear_range', topic_mm='/robotek/rear_range_mm').items()}
        self.bus, self.address = int(p['bus']), int(p['address'])
        self.min_m, self.max_m, self.fov = float(p['min_m']), float(p['max_m']), float(p['fov_rad'])
        self.frame_id = str(p['frame_id'])
        self.correct = bool(p['correct'])
        self.table = list(zip((float(x) for x in p['table_raw_mm']), (float(x) for x in p['table_mm'])))
        if self.correct and (len(self.table) < 2 or len(p['table_raw_mm']) != len(p['table_mm'])
                             or any(b[0] <= a[0] for a, b in zip(self.table, self.table[1:]))):
            raise ValueError('table_raw_mm and table_mm need the same length (>= 2) and table_raw_mm must increase')
        self.pub = self.create_publisher(Range, str(p['topic']), 10)
        self.pub_mm = self.create_publisher(Int32, str(p['topic_mm']), 10)
        self.i2c = None
        self.dev = None
        self.waited = 0
        self.failures = 0
        self.readings = 0
        if lgpio is None:
            self.get_logger().error('lgpio missing (sudo apt install python3-lgpio): rear range disabled.')
            return
        self.open_i2c()
        self.create_timer(5., self.retry_i2c)   # also re-initialises a sensor that was dropped (see poll)
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
        dev = Vl53l0x(handle)
        try:
            dev.init()
            dev.start_continuous()
        except Exception as error:  # noqa: BLE001  lgpio.error, TimeoutError
            self.close(handle)
            self.get_logger().warn(f'VL53L0X init failed ({error}): retrying every 5 s.', throttle_duration_sec=60.)
            return False
        self.i2c, self.dev, self.waited = handle, dev, 0
        self.get_logger().info(f'VL53L0X on I2C bus {self.bus} address 0x{self.address:02x}: initialised, '
                               'continuous ranging.')
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
        if self.dev is None:
            return
        try:
            if not self.dev.ready():
                self.waited += 1
                if self.waited > 50:    # 0.5 s without a result: the sensor may have reset (power or cable)
                    self.drop('VL53L0X gave no result for 0.5 s')
                return
            reading = self.dev.result()
        except Exception as error:  # noqa: BLE001
            self.failures += 1
            self.get_logger().warn(f'I2C read failed ({error}).', throttle_duration_sec=10.)
            if self.failures >= 10:
                self.drop('10 I2C errors in a row')
            return
        self.waited = self.failures = 0
        self.publish(*reading)

    def drop(self, why):
        """Forget the sensor; retry_i2c opens it again and runs the whole init within 5 s. A VL53L0X that lost power
        comes back with its factory registers, so restarting the ranging alone is not enough (9 nov 2026)."""
        self.get_logger().warn(f'{why}: re-initialising the sensor.')
        self.close(self.i2c)
        self.i2c, self.dev, self.waited, self.failures = None, None, 0, 0

    def publish(self, raw, status):
        mm = corrected(raw, self.table) if self.correct else raw
        valid = status == STATUS_VALID and 0 < raw < NO_TARGET_MM
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
            self.get_logger().info(f'First reading: {mm} mm (raw {raw} mm, status {status}).')

    def destroy_node(self):
        if self.dev is not None:
            try:
                self.dev.stop()
            except Exception:  # noqa: BLE001
                pass
            self.dev = None
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
