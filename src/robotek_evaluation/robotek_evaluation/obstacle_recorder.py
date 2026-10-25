"""Audit-only recorder of the Obstacle rounds: contacts and the pose of the signs.

Never publishes. Writes ``<output_dir>/contacts.jsonl``, one line per contact episode of the car (``contact_log.py``)
with anything but the floor, from the sensors on the signs and the lot limits (``/obstacle/contacts``) and on the
walls (``/audit/robot_contacts``); each line has ``pillar`` (the sign, or null) and ``kind`` (outer_wall, inner_wall,
pillar, parking_limit, other). Also ``pillars-gt.jsonl``, the pose of every sign 20 times a second
(``/obstacle/pillars_gt``), for the judge (a sign moved out of its circle ends the round), and
``contacts-health.json``. The strategy never sees any of it.
"""
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from tf2_msgs.msg import TFMessage

from robotek_evaluation.contact_log import EpisodeLog

GROUND = ('ground_plane', 'game_mat', '::mat', 'mat::')


def kind_of(name):
    if 'wall_ext' in name:
        return 'outer_wall'
    if 'wall_int' in name:
        return 'inner_wall'
    if 'sign_' in name:
        return 'pillar'
    if 'parking' in name or 'magenta' in name:
        return 'parking_limit'
    return 'other'


class ObstacleRecorder(Node):
    def __init__(self):
        super().__init__('obstacle_recorder')
        directory = Path(self.declare_parameter('output_dir', '').value).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        self.directory = directory
        self.out = (directory/'contacts.jsonl').open('w')
        self.contacts = EpisodeLog(self.out)
        self.pillars = (directory/'pillars-gt.jsonl').open('w')
        self.create_subscription(TFMessage, '/obstacle/pillars_gt', self.on_pillars, 10)
        # Health: proves the sensors publish even when no robot contact is kept.
        self.health = {}
        for topic in ('/obstacle/contacts', '/audit/robot_contacts'):
            self.create_subscription(Contacts, topic,
                                     lambda msg, t=topic: self.on_contacts(msg, t), 50)

    def write_health(self):
        (self.directory/'contacts-health.json').write_text(json.dumps(self.health, indent=2)+'\n')

    def on_pillars(self, msg):
        def yaw_of(q):
            return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.pillars.write(json.dumps(dict(
            sim_time=self.get_clock().now().nanoseconds*1e-9, poses=[
                dict(name=t.child_frame_id, frame=t.header.frame_id,
                     x=t.transform.translation.x, y=t.transform.translation.y,
                     z=t.transform.translation.z, yaw=yaw_of(t.transform.rotation),
                     q=[t.transform.rotation.x, t.transform.rotation.y,
                        t.transform.rotation.z, t.transform.rotation.w])
                for t in msg.transforms]))+'\n')

    def on_contacts(self, msg, topic):
        h = self.health.setdefault(topic, dict(messages=0, contacts=0, kept=0, ground=0))
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
            pillar = other[0].split('::')[1] if 'sign_' in other[0] else None
            self.contacts.add(dict(
                sim_time=self.get_clock().now().nanoseconds*1e-9, wall_time=time.time(),
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9, source=topic,
                pillar=pillar, other=other[0], kind=kind_of(other[0]), robot=robot[0],
                max_depth=max(c.depths, default=None), points=len(c.positions)))
        self.out.flush()


def main():
    rclpy.init()
    node = ObstacleRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        # SIGINT during shutdown can interrupt take_message inside rclpy.
        pass
    finally:
        node.contacts.close()
        node.out.close()
        node.pillars.close()
        node.write_health()
