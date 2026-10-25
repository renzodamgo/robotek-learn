"""Audit-only recorder: contacts of the car with the field walls, from Gazebo's contact sensor.

The world has a ``wall_contact`` sensor on the walls; the launch bridges its topic (/audit/robot_contacts). This node
never publishes. It writes ``<output_dir>/contacts.jsonl``, one line per contact episode (``contact_log.py``), and
``contacts-health.json``, which proves the sensor published even when nothing was touched.
"""
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts

from robotek_evaluation.contact_log import EpisodeLog

GROUND = ('ground_plane', 'game_mat', '::mat', 'mat::')
TOPIC = '/audit/robot_contacts'


def kind_of(name):
    if 'wall_ext' in name:
        return 'outer_wall'
    if 'wall_int' in name:
        return 'inner_wall'
    return 'other'


class ContactRecorder(Node):
    def __init__(self):
        super().__init__('contact_recorder')
        directory = Path(self.declare_parameter('output_dir', '').value).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        self.directory = directory
        self.out = (directory / 'contacts.jsonl').open('w')
        self.contacts = EpisodeLog(self.out)
        self.health = dict(messages=0, contacts=0, kept=0, ground=0)
        self.create_subscription(Contacts, TOPIC, self.on_contacts, 50)

    def write_health(self):
        (self.directory / 'contacts-health.json').write_text(json.dumps({TOPIC: self.health}, indent=2) + '\n')

    def on_contacts(self, msg):
        h = self.health
        h['messages'] += 1
        h['contacts'] += len(msg.contacts)
        for c in msg.contacts:
            names = (c.collision1.name, c.collision2.name)
            robot = [n for n in names if n.startswith('robotek::')]
            other = [n for n in names if not n.startswith('robotek::')]
            if not robot or not other or any(g in other[0] for g in GROUND):
                if robot and other:
                    h['ground'] += 1
                continue
            h['kept'] += 1
            self.contacts.add(dict(
                sim_time=self.get_clock().now().nanoseconds * 1e-9, wall_time=time.time(),
                stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, source=TOPIC,
                pillar=None, other=other[0], kind=kind_of(other[0]), robot=robot[0],
                max_depth=max(c.depths, default=None), points=len(c.positions)))
        self.out.flush()


def main():
    rclpy.init()
    node = ContactRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass    # SIGINT during shutdown can interrupt take_message inside rclpy
    finally:
        node.contacts.close()
        node.out.close()
        node.write_health()
