"""Record one round of the Open Challenge and decide when it is over.

The strategy only drives. This node is the referee's assistant:

* waits until the sensors and the strategy are up, then publishes ``/start`` and writes the Start event;
* records the true pose of the car (``audit_pose.csv``: the strategy never sees it, only the judge does);
* ends the round after 180 s, after 30 s without moving to another mat section, or when the car has driven
  the three laps and stays still, and then scores it (``score.py``).

Everything goes to ``<output_dir>``: ``audit_pose.csv``, ``events.jsonl``, ``manifest.json``, ``world.sdf``
(the field of the round) and, at the end, ``rule_judge.json``.
"""
import csv
import datetime
import json
import math
import shutil
import time
from pathlib import Path

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Bool

from robotek_evaluation.open_field_geometry import field_geometry
from robotek_evaluation.rule_judge import ROUND_LIMIT_S, STOP_OBSERVED_S, mat_section

from .score import STILL_M, format_report, score_directory

SETTLE_S = 1.0            # simulated seconds with every sensor alive before Start
STALL_S = 30.0            # no new mat section for this long: the round cannot score any more
REST_S = STOP_OBSERVED_S + 1.0
MIN_EXITS_TO_FINISH = 23  # centre-based section exits (the judge counts the footprint, which lags a little)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class RoundRecorder(Node):
    def __init__(self):
        super().__init__('round_recorder')
        directory = Path(self.declare_parameter('output_dir', '').value).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        self.directory = directory
        self.sign = 1 if self.declare_parameter('direction', 'ccw').value == 'ccw' else -1
        self.limit = float(self.declare_parameter('round_limit', ROUND_LIMIT_S).value)
        self.write_manifest()

        self.file = (directory / 'audit_pose.csv').open('w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['wall_time', 'sim_time', 'x_gt', 'y_gt', 'yaw_gt', 'steer_actual', 'steer_right_actual'])
        self.events = (directory / 'events.jsonl').open('w')

        self.steer = (math.nan, math.nan)
        self.alive = {'pose': None, 'scan': None, 'imu': None}   # sim time of the first message of each sensor
        self.start = None                                        # sim time of Start
        self.pose = None
        self.section = self.section_since = None
        self.exits = self.net = 0
        self.rest_ref = self.rest_since = None
        self.finished = False

        self.start_pub = self.create_publisher(
            Bool, 'start', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, '/odom_gt', self.on_pose, 50)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 50)
        self.create_subscription(LaserScan, '/scan', lambda m: self.mark_alive('scan'), qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', lambda m: self.mark_alive('imu'), qos_profile_sensor_data)
        self.create_timer(0.1, self.supervise)

    # ----------------------------------------------------------------------------------------- files
    def write_manifest(self):
        world = Path(self.declare_parameter('world', '').value)
        manifest = dict(
            direction='ccw' if self.sign == 1 else 'cw', started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
            strategy=self.declare_parameter('strategy', '').value, honest=True, real_car=True,
            judge='robotek_course.round_recorder')
        if world.is_file():
            shutil.copyfile(world, self.directory / 'world.sdf')
            manifest['field_geometry'] = field_geometry(world)
        (self.directory / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')

    def event(self, name, **fields):
        self.events.write(json.dumps(dict(event=name, source='round_recorder', sim_time=self.now(),
                                          wall_time=time.time(), **fields)) + '\n')
        self.events.flush()

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------------------------ callbacks
    def mark_alive(self, name):
        if self.alive[name] is None:
            self.alive[name] = self.now()

    def on_joints(self, msg):
        values = dict(zip(msg.name, msg.position))
        self.steer = (values.get('front_left_steering_joint', math.nan),
                      values.get('front_right_steering_joint', math.nan))

    def on_pose(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_of(msg.pose.pose.orientation))
        self.mark_alive('pose')
        if not self.finished:
            self.writer.writerow([time.time(), self.now(), *self.pose, *self.steer])

    # ------------------------------------------------------------------------------------ the round
    def supervise(self):
        if self.finished or self.pose is None:
            return
        now = self.now()
        if self.start is None:
            ready = all(t is not None for t in self.alive.values()) and now - max(self.alive.values()) >= SETTLE_S
            if ready and self.start_pub.get_subscription_count() == 0:
                self.get_logger().warning('Waiting for your node to subscribe to /start (is it running?)',
                                          throttle_duration_sec=10.0)
            if ready and self.start_pub.get_subscription_count() > 0:
                self.start = now
                self.event('start')
                self.start_pub.publish(Bool(data=True))
                self.get_logger().info('Start')
            return
        self.track_progress(now)
        if now - self.start >= self.limit:
            self.end('round_time_limit_180s')
        elif self.exits >= MIN_EXITS_TO_FINISH and now - self.rest_since >= REST_S:
            self.end('stopped_after_three_laps')
        elif self.exits < MIN_EXITS_TO_FINISH and now - self.section_since >= STALL_S:
            self.end('no_section_progress_%ds' % STALL_S)

    def track_progress(self, now):
        x, y, _ = self.pose
        section = mat_section(x, y)
        if self.section is None:
            self.section, self.section_since = section, now
        elif section != self.section:
            if section == (self.section + self.sign) % 8:
                self.net += 1
            elif section == (self.section - self.sign) % 8:
                self.net -= 1
            self.exits = max(self.exits, self.net)
            self.section, self.section_since = section, now
        if self.rest_ref is None or math.hypot(x - self.rest_ref[0], y - self.rest_ref[1]) > STILL_M:
            self.rest_ref, self.rest_since = (x, y), now

    def end(self, reason):
        self.finished = True
        self.event('end', reason=reason)
        self.file.close()
        self.events.close()
        print('\nRonda terminada (%s).\n' % reason, flush=True)
        try:
            print(format_report(self.directory, score_directory(self.directory)), flush=True)
        except Exception as error:   # the files stay on disk: score can be run again by hand
            print('No se pudo puntuar la ronda: %r\nPuedes repetirlo con: ros2 run robotek_course score %s'
                  % (error, self.directory), flush=True)
        rclpy.shutdown()


def main():
    rclpy.init()
    node = RoundRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        if not node.finished:
            node.file.close()
            node.events.close()
