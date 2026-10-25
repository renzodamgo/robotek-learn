"""Warns when the battery should be changed (default below 6.8 V); it never stops the car.

Reads /rrc/battery (mV, ~1 Hz) and, after `confirm_samples` consecutive
readings under `min_voltage`, publishes /robotek/battery_low (Bool true, latched): status_leds
then blinks both board RGBs red (the buzzer is burnt out). Latched: it stays on until the launch
is restarted with a charged battery, so a warning seen during a round is still there after it.

6.8 V = 3.4 V per cell of the 2S LiPo: little charge left, change it before the next round.
The cells are damaged below ~3.0 V (6.0 V), which is also the board's minimum input. Several
readings in a row are needed so a single odd reading (the servo turning, a board glitch) does not
trip it. The drive motor gets its own battery (team decision, 9 Oct 2026), so this one only
feeds the electronics and does not sag with the motor; the motor battery is not monitored.
(9 Oct 2026: changed from stopping the car at 6.6 V to only warning, at the team's request.)

  ros2 launch robotek_course car.launch.py strategy:=my_open                    # on (6.8 V)
  ros2 launch robotek_course car.launch.py strategy:=my_open battery_guard:=false   # off
  ros2 launch robotek_course car.launch.py strategy:=my_open battery_min_v:=7.0     # other limit
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, UInt16

from .mapping import LowVoltage


class BatteryGuard(Node):
    def __init__(self):
        super().__init__('battery_guard')
        min_v = float(self.declare_parameter('min_voltage', 6.8).value)
        confirm = int(self.declare_parameter('confirm_samples', 5).value)
        self.low = LowVoltage(min_v, confirm)
        self.warned = False
        self.low_pub = self.create_publisher(
            Bool, '/robotek/battery_low', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(UInt16, '/rrc/battery', self.on_battery, 10)
        self.get_logger().info(f'Battery guard: red blinking below {min_v:.2f} V '
                               f'({confirm} consecutive readings); it does not stop the car.')

    def on_battery(self, msg):
        volts = msg.data/1000.
        if self.low.update(volts) and not self.warned:
            self.warned = True
            self.low_pub.publish(Bool(data=True))
            self.get_logger().warning(f'Battery {volts:.2f} V < {self.low.min_v:.2f} V: '
                                      'change it before the next round (the car keeps going).')


def main(args=None):
    rclpy.init(args=args)
    node = BatteryGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl+C in Jazzy
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
