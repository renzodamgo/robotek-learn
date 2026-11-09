"""Push button on a Raspberry Pi GPIO -> /robotek/gpio_button (std_msgs/Bool): True when pressed, False when
released, published on every debounced change, with a log line.

Wiring (default): one leg on GPIO 16 (BCM, physical pin 36), the other on GND; the internal pull-up keeps the pin
high, a press pulls it low. pull:=down (button to 3.3 V) or pull:=none (external resistor) flip it; active_low
says which level means pressed (default: low when the pull is up, high when it is down).

The pin is polled every poll_s (10 ms) and a change counts once it has held for debounce_s (30 ms). On a Mac or
without lgpio the node logs it and keeps retrying, it never crashes the launch."""
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    import lgpio
except ImportError:  # only on the Raspberry Pi (python3-lgpio)
    lgpio = None

PULLS = {'up': 'SET_PULL_UP', 'down': 'SET_PULL_DOWN', 'none': 'SET_PULL_NONE'}


class GpioButton(Node):
    def __init__(self):
        super().__init__('gpio_button')
        p = {k: self.declare_parameter(k, v).value for k, v in dict(
            gpiochip=4, pin=16, pull='up', active_low='auto', debounce_s=.03, poll_s=.01,
            topic='/robotek/gpio_button').items()}
        self.chip, self.pin, self.pull = int(p['gpiochip']), int(p['pin']), str(p['pull']).lower()
        if self.pull not in PULLS:
            raise ValueError(f'pull must be one of {sorted(PULLS)}, not {self.pull!r}')
        low = str(p['active_low']).lower()
        self.active_low = self.pull != 'down' if low == 'auto' else low in ('1', 'true', 'yes')
        self.debounce = float(p['debounce_s'])
        self.pub = self.create_publisher(Bool, str(p['topic']), 10)
        self.gpio = None
        self.pressed = None      # debounced state; None until the first read
        self.candidate = None    # raw state waiting to hold for debounce_s
        self.since = 0.
        self.presses = 0
        if lgpio is None:
            self.get_logger().error('lgpio missing (sudo apt install python3-lgpio): button disabled.')
            return
        if not self.open_gpio():
            self.create_timer(5., self.retry_gpio)
        self.create_timer(float(p['poll_s']), self.poll)

    def open_gpio(self):
        handle = None
        try:
            handle = lgpio.gpiochip_open(self.chip)
            lgpio.gpio_claim_input(handle, self.pin, getattr(lgpio, PULLS[self.pull]))
        except Exception as error:  # lgpio.error: no such chip, permission denied, pin busy
            if handle is not None:
                try:
                    lgpio.gpiochip_close(handle)
                except Exception:  # noqa: BLE001
                    pass
            self.get_logger().warn(f'GPIO chip {self.chip} pin {self.pin} not usable ({error}): retrying every 5 s.',
                                   throttle_duration_sec=60.)
            return False
        self.gpio = handle
        level = lgpio.gpio_read(handle, self.pin)
        self.get_logger().info(
            f'Button on GPIO {self.pin} (chip {self.chip}, pull {self.pull}, pressed = '
            f'{"low" if self.active_low else "high"}); level now {level} = '
            f'{"PRESSED" if self.is_pressed(level) else "released"}.')
        return True

    def retry_gpio(self):
        if self.gpio is None:
            self.open_gpio()

    def is_pressed(self, level):
        return (level == 0) == self.active_low

    def poll(self):
        if self.gpio is None:
            return
        try:
            raw = self.is_pressed(lgpio.gpio_read(self.gpio, self.pin))
        except Exception as error:  # noqa: BLE001
            self.get_logger().warn(f'GPIO read failed ({error}).', throttle_duration_sec=10.)
            return
        now = time.monotonic()
        if self.pressed is None:
            self.pressed = raw
            return
        if raw == self.pressed:
            self.candidate = None
            return
        if raw != self.candidate:
            self.candidate, self.since = raw, now
            return
        if now-self.since < self.debounce:
            return
        self.pressed, self.candidate = raw, None
        self.pub.publish(Bool(data=raw))
        if raw:
            self.presses += 1
            self.get_logger().info(f'Button PRESSED (#{self.presses}).')
        else:
            self.get_logger().info('Button released.')

    def destroy_node(self):
        if self.gpio is not None:
            try:
                lgpio.gpio_free(self.gpio, self.pin)
                lgpio.gpiochip_close(self.gpio)
            except Exception:  # noqa: BLE001
                pass
            self.gpio = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpioButton()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
