"""base_adapter done_reset_states: on DONE it sends 0, then resets the board once, then no motor command."""
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String

from robotek_hardware import base_adapter


def test_done_sends_zero_then_resets_once():
    rclpy.init()
    try:
        node = base_adapter.BaseAdapter.__new__(base_adapter.BaseAdapter)
        base_adapter.BaseAdapter.__init__(node)
        if base_adapter.MotorsState is None:          # rrc_lite_msgs not built here: the rrc path cannot run
            node.destroy_node(); return
        node.backend, node.done_reset_states, node.motor_halted = 'rrc', {'DONE'}, False
        sent, resets = [], []
        node.motor_pub = type('P', (), {'publish': lambda self, m: sent.append(m.data[0].rps)})()
        node.reset_pub = type('P', (), {'publish': lambda self, m: resets.append(1)})()
        node.on_state(String(data='FINISH'))
        node.on_state(String(data='DONE'))
        node.on_state(String(data='DONE'))
        node.send_speed(0.5)
        assert sent and sent[-1] == 0.0 and resets == [1]
        node.destroy_node()
    finally:
        rclpy.try_shutdown()
