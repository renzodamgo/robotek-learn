"""rear_range with a fake lgpio: the init sequence runs, a valid reading publishes metres and mm, 'no target' and a
bad status publish inf and -1, and a sensor with another model id is refused."""
import math
import types

import rclpy

from robotek_hardware import rear_range


def block(mm, status=11):
    data = bytearray(12)
    data[0] = status << 3
    data[10], data[11] = mm >> 8, mm & 0xFF
    return data


def make_node(reading, model=0xEE):
    fake = types.SimpleNamespace(
        i2c_open=lambda bus, address: 3, i2c_close=lambda h: 0,
        i2c_read_byte_data=lambda h, reg: model if reg == rear_range.MODEL_ID else 0x01,
        i2c_write_byte_data=lambda h, reg, value: 0,
        i2c_write_i2c_block_data=lambda h, reg, data: 0,
        i2c_read_i2c_block_data=lambda h, reg, count: (count, reading[0]))
    rear_range.lgpio = fake
    return rear_range.RearRange()


def test_readings():
    rclpy.init()
    try:
        reading = [block(253)]
        node = make_node(reading)
        ranges, mms = [], []
        node.pub.publish = lambda msg: ranges.append(msg.range)
        node.pub_mm.publish = lambda msg: mms.append(msg.data)
        assert node.dev is not None     # init and continuous start went through
        node.poll()
        reading[0] = block(8190)
        node.poll()
        reading[0] = block(120, status=4)   # phase failure
        node.poll()
        assert mms == [145, -1, -1]           # (253 - 44) / 1.44
        assert math.isclose(ranges[0], .145, abs_tol=1e-6) and math.isinf(ranges[1]) and math.isinf(ranges[2])
        node.destroy_node()
    finally:
        rclpy.try_shutdown()


def test_other_model_refused():
    rclpy.init()
    try:
        node = make_node([block(100)], model=0xEA)
        assert node.i2c is None and node.dev is None
        node.destroy_node()
    finally:
        rclpy.try_shutdown()
