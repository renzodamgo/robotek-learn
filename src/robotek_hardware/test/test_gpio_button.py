"""gpio_button debouncing with a fake lgpio: a 10 ms bounce is ignored, a held press and release publish once each."""
import types

import rclpy

from robotek_hardware import gpio_button


def make_node(levels):
    fake = types.SimpleNamespace(
        SET_PULL_UP=1, SET_PULL_DOWN=2, SET_PULL_NONE=3,
        gpiochip_open=lambda chip: 7, gpio_claim_input=lambda h, pin, flags: 0,
        gpio_read=lambda h, pin: levels[0], gpio_free=lambda h, pin: 0, gpiochip_close=lambda h: 0)
    gpio_button.lgpio = fake
    return gpio_button.GpioButton()


def test_debounce(monkeypatch):
    rclpy.init()
    try:
        levels = [1]
        node = make_node(levels)
        sent = []
        node.pub.publish = lambda msg: sent.append(msg.data)
        clock = [0.]
        monkeypatch.setattr(gpio_button.time, 'monotonic', lambda: clock[0])

        def run(level, seconds):
            levels[0] = level
            for _ in range(round(seconds/.01)):
                node.poll()
                clock[0] += .01
        run(1, .1)          # released at start: nothing
        run(0, .01)         # bounce
        run(1, .1)
        assert sent == []
        run(0, .2)          # held press
        run(1, .2)          # release
        assert sent == [True, False]
        assert node.presses == 1
        node.destroy_node()
    finally:
        rclpy.try_shutdown()
