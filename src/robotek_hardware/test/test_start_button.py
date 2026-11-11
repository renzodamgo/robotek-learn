"""start_button: the GPIO 16 button starts, a press after the lockout stops; the board's button is off by default."""
import rclpy
from std_msgs.msg import Bool

from robotek_hardware import start_button


def test_gpio_press_starts_then_stops():
    rclpy.init()
    try:
        node = start_button.StartButton()
        starts, stops = [], []
        node.pub.publish = lambda m: starts.append(m.data)
        node.stop_pub.publish = lambda m: stops.append(m.data)
        times = iter([0.0, 0.5, 10.0])
        node.get_clock = lambda: type('C', (), {'now': lambda self: type('T', (), {'nanoseconds': int(next(times) * 1e9)})()})()
        node.on_gpio(Bool(data=True))     # start
        node.on_gpio(Bool(data=False))    # release: nothing
        node.on_gpio(Bool(data=True))     # 0.5 s later: debounced
        node.on_gpio(Bool(data=True))     # 10 s later: stop
        assert starts == [True] and stops == [True]
        node.destroy_node()
    finally:
        rclpy.try_shutdown()
