"""The ROS node of Obstacle v3: sensors in, /cmd_vel_raw out. The decisions are in strategy.py and mission.py.

The same topics as a student's ``my_obstacle`` node, so it runs in ``obstacle_round.launch.py`` and
``car.launch.py`` with ``strategy:=robotek_obstacle_v3``. Two things differ from the course's ``drive.py``, both
taken from the v3 node of WRO-Strategies so the decisions stay the same:

* every camera frame reaches the strategy: one with no scan within ``IMAGE_PAIRING`` gives an empty list of signs
  (the course's node skips it). The strategy needs a fresh frame to keep driving;
* the 35 mission parameters of ``settings.Params`` are ROS parameters (defaults = WRO-Strategies'
  ``obstacle_v3.yaml``); ``parking`` (from the round launch) sets ``park``.

The camera is ROS parameters too, for a camera other than the one of the defaults (the USB camera measured on
8 oct 2026; ``car.launch.py strategy_params:=<file.yaml>`` sets them): ``camera_hfov``, ``camera_pitch`` and
``camera_yaw`` (rad, + = looks left of the LiDAR's zero) turn an image column into a bearing, and ``red_hsv`` /
``green_hsv`` are the colours of the signs as ``"h_min,h_max,s_min,v_min"`` (h_min > h_max wraps through 0, as red
does). ``sign_min_area`` is the smallest blob in pixels.

For RViz, only while someone watches them: ``/obstacle/debug_image/compressed`` (the frame with the red and green
blobs, thick when paired with a LiDAR cluster, the state and the steering) and ``/obstacle/signs`` (the paired signs
as cylinders and the steering as an arrow, in base_link).
"""
from dataclasses import asdict
import math
import os
import time

import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, Imu, LaserScan
from std_msgs.msg import Bool, Float32MultiArray, Int32, String
from visualization_msgs.msg import Marker, MarkerArray

from robotek_common.vehicle import REAL_CAMERA_HFOV, REAL_CAMERA_PITCH, REAL_CAMERA_YAW, WHEELBASE
from robotek_course.obstacle_vision import COLOR_RANGES, associate, color_blocks

from .lot_exit import DEFAULT_STEPS, parse_steps
from .perception import scan_clusters
from .recording import Recording, default_folder, sector_min
from . import scene
from .field import SWAP
from .mission import SEAT_HITS
from .scene import UNITS_PER_LAP
from .settings import Params
from .strategy import Strategy

CONTROL_RATE = 50.0       # Hz, on the node's clock (simulation clock in Gazebo)
IMAGE_PAIRING = 0.15      # s; a camera frame is paired with a scan at most this old
DEBUG_PERIOD = 0.2        # s; the detections image at most 5 times a second (JPEG at 25 fps made the camera stale, 8 nov)


def hsv_bands(spec):
    """``"h_min,h_max,s_min,v_min"`` -> the [(low, high)] bands of ``color_blocks``; h_min > h_max wraps through 0."""
    h_min, h_max, s_min, v_min = (int(float(v)) for v in spec.split(','))
    if h_min <= h_max:
        return [((h_min, s_min, v_min), (h_max, 255, 255))]
    return [((0, s_min, v_min), (h_max, 255, 255)), ((h_min, s_min, v_min), (179, 255, 255))]


def hsv_spec(bands):
    """The inverse of ``hsv_bands`` for the default ranges (used as the parameters' defaults)."""
    (low, high), *rest = bands
    if rest:                                   # wrapped: (0..h_max) and (h_min..179)
        return '%d,%d,%d,%d' % (rest[-1][0][0], high[0], low[1], low[2])
    return '%d,%d,%d,%d' % (low[0], high[0], low[1], low[2])


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Drive(Node):
    def __init__(self):
        super().__init__('strategy')
        parking = self.declare_parameter('parking', False).value
        defaults = asdict(Params())
        values = {name: float(self.declare_parameter(name, float(value)).value) for name, value in defaults.items()}
        # Only the parameters changed by hand: a default park=0 must not cancel parking:=true.
        params = {name: value for name, value in values.items() if value != defaults[name]}
        self.strategy = Strategy(self.declare_parameter('direction', 'auto').value, parking=parking, params=params)
        steps = str(self.declare_parameter('lot_exit_steps', DEFAULT_STEPS).value)
        parse_steps(steps)                               # a bad sequence stops the node at launch, not in the lot
        self.strategy.mission.lot_exit_steps = steps
        finish = str(self.declare_parameter('park_finish_steps', '').value).strip()
        if finish:
            parse_steps(finish)                          # a bad sequence stops the node at launch
        self.strategy.mission.park_finish_steps = finish
        self.strategy.auto_start = False                 # the referee (or the button on the car) publishes /start
        self.camera = dict(
            hfov=float(self.declare_parameter('camera_hfov', REAL_CAMERA_HFOV).value),
            pitch=float(self.declare_parameter('camera_pitch', REAL_CAMERA_PITCH).value),
            yaw=float(self.declare_parameter('camera_yaw', REAL_CAMERA_YAW).value),
            ranges={color: hsv_bands(self.declare_parameter(color + '_hsv', hsv_spec(COLOR_RANGES[color])).value)
                    for color in ('red', 'green')},
            min_area=int(self.declare_parameter('sign_min_area', 30).value))
        self.bridge = CvBridge()
        self.scan_msg = None
        self.cmd = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.estop = self.create_publisher(Bool, '/robotek/estop', 10)   # the lot exit's hard brake
        self.lot_logged = 0          # lot exit log entries already printed
        self.events_logged = 0       # recover log entries already printed
        self.park_state = None     # last parking sub-state logged
        self.state = self.create_publisher(
            String, '/robotek/state', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.state_sent = None
        self.pose_pub = self.create_publisher(Float32MultiArray, '/obstacle/pose', 10)   # x, y, th, progress
        self.debug_pub = self.create_publisher(CompressedImage, '/obstacle/debug_image/compressed', 1)
        self.signs_pub = self.create_publisher(MarkerArray, '/obstacle/signs', 1)
        self.steer = 0.0
        self.debug_sent = -1e9
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        # camera_compressed (car.launch with the picam): the JPEG of the frame, not the raw image. 12 nov 2026 on the Pi
        # with 3 cores busy: of 27.6 raw frames/s (691 KB each) 0.2/s reached another process (best effort; 5/s
        # reliable), of the JPEG at quality 90 (82 KB) all 30/s. In the 01:05 round the camera "stopped" at 14.5 s.
        if self.declare_parameter('camera_compressed', False).value:
            self.create_subscription(CompressedImage, '/camera/image/compressed', self.on_compressed,
                                     qos_profile_sensor_data)
        else:
            self.create_subscription(Image, '/camera/image', self.on_image, qos_profile_sensor_data)
        self.create_subscription(Bool, 'start', self.on_start, 10)   # plain QoS: the referee and the car's button
        # The push button on the Pi's GPIO 16 (gpio_button node, True on each press) also starts the round (9 nov 2026).
        self.create_subscription(Bool, '/robotek/gpio_button', self.on_start, 10)
        # Rear VL53L0X (rear_range node, mm, -1 = nothing in range): the lot exit backs up on it (start_lot).
        self.create_subscription(Int32, '/robotek/rear_range_mm', self.on_rear, 10)
        # What the round leaves behind (recording.py), as the Open v5: 'auto' = ~/runs/<date>-<time>-obstacle-v3/,
        # '' or 'off' = nothing; telemetry_every = control ticks per telemetry row (2 = 25 Hz).
        folder = str(self.declare_parameter('telemetry', 'auto').value)
        folder = default_folder() if folder == 'auto' else ('' if folder in ('', 'off') else os.path.expanduser(folder))
        self.rec = Recording(folder, every=int(self.declare_parameter('telemetry_every', 2).value))
        self.rec.params(params, values)
        self.rear_mm = -1
        self.t_start = None          # node clock at Start (the button or /start)
        self.corners_logged = 0      # corners already announced (progress units: odd = corner)
        self.laps_logged, self.lap_start = 0, 0.0
        self.laps = []               # lap times
        self.section, self.section_start = None, 0.0   # current section (floor of the progress) and its start time
        self.sections = []           # (section, kind, seconds)
        self.signs_known = {}        # seat -> colour announced (None = seen, colour unknown)
        self.signs_passed = set()    # (lap, seat) already reported as passed
        self.round_logged = False
        self.camera_lost_logged = False
        self.create_timer(1.0 / CONTROL_RATE, self.control)
        p = self.strategy.mission.p
        self.get_logger().info('Obstacle v3 ready (park=%g, steer_limit=%.3f rad, changed: %s; camera %s)'
                               % (p.park, p.steer_limit, params, self.camera))
        self.get_logger().info('Telemetry: %s' % (folder or 'off'))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_scan(self, msg):
        self.scan_msg = msg
        self.rec.scan(msg)
        self.strategy.on_scan(msg)

    def on_imu(self, msg):
        self.strategy.on_imu(stamp_of(msg), msg.angular_velocity.z)

    def on_compressed(self, msg):
        """The JPEG frame: decoded only when it pairs with a scan, then as on_image."""
        self.on_image(msg, decode=lambda: cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR))

    def on_image(self, msg, decode=None):
        """Red and green blocks of the frame paired with the LiDAR clusters at the same bearing."""
        stamp = stamp_of(msg)
        observations = []
        if self.scan_msg is not None and abs(stamp - stamp_of(self.scan_msg)) <= IMAGE_PAIRING:
            bgr = decode() if decode else self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            blocks = color_blocks(bgr, **self.camera)
            observations = associate(blocks, scan_clusters(self.scan_msg))
            if self.debug_pub.get_subscription_count() and stamp - self.debug_sent >= DEBUG_PERIOD:
                self.debug_sent = stamp
                self.publish_debug(msg, bgr, blocks, observations)
        self.strategy.on_signs(observations, stamp)
        if observations:
            self.rec.signs(stamp, observations)
        if self.signs_pub.get_subscription_count():
            self.publish_signs(msg, observations)

    def publish_debug(self, msg, bgr, blocks, observations):
        """The frame with every red and green blob (thick = paired with a LiDAR cluster), the state and the steering."""
        paired = {o['box'] for o in observations}
        for block in blocks:
            x, y, w, h = block['box']
            color = (0, 200, 0) if block['color'] == 'green' else (0, 0, 255)
            thick = 3 if block['box'] in paired else 1
            cv2.rectangle(bgr, (x, y), (x + w, y + h), color, thick)
            cv2.putText(bgr, '%s %+.0f' % (block['color'][0].upper(), math.degrees(block['bearing'])), (x, max(12, y - 3)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        for o in observations:
            cv2.putText(bgr, '%s %.2f,%+.2f' % (o['color'][0].upper(), o['x'], o['y']), (8, bgr.shape[0] - 10 - 16 *
                        observations.index(o)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
        cv2.putText(bgr, '%s  signs %d  steer %+.0f' % (self.strategy.label, len(observations), math.degrees(self.steer)),
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
        out = CompressedImage(format='jpeg', data=cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes())
        out.header = msg.header
        self.debug_pub.publish(out)

    def publish_signs(self, msg, observations):
        """The paired signs as cylinders and the commanded steering as an arrow, in base_link."""
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for i, o in enumerate(observations):
            m = Marker(ns='signs', id=i, type=Marker.CYLINDER, action=Marker.ADD)
            m.header.frame_id, m.header.stamp = 'base_link', msg.header.stamp
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(o['x']), float(o['y']), 0.05
            m.scale.x = m.scale.y = 0.05
            m.scale.z = 0.1
            m.color.a, m.color.r, m.color.g = 0.9, float(o['color'] == 'red'), float(o['color'] == 'green')
            markers.markers.append(m)
        arrow = Marker(ns='steer', id=0, type=Marker.ARROW, action=Marker.ADD)
        arrow.header.frame_id, arrow.header.stamp = 'base_link', msg.header.stamp
        arrow.pose.position.x = float(WHEELBASE)
        arrow.pose.orientation.z, arrow.pose.orientation.w = math.sin(self.steer / 2), math.cos(self.steer / 2)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.25, 0.02, 0.02
        arrow.color.a, arrow.color.r, arrow.color.g, arrow.color.b = 1.0, 1.0, 1.0, 0.0
        markers.markers.append(arrow)
        self.signs_pub.publish(markers)

    def on_rear(self, msg):
        self.rear_mm = int(msg.data)
        self.strategy.mission.rear(self.now(), self.rear_mm)

    def on_start(self, msg):
        if msg.data and not self.strategy.started:
            self.get_logger().info('Start.')
            self.t_start = self.now()
            self.rec.event('start', 0.0)
            self.strategy.request_start()

    def control(self):
        speed, steer = self.strategy.control(self.now(), 1.0 / CONTROL_RATE)
        self.steer = steer
        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(speed * math.tan(steer) / WHEELBASE)
        self.cmd.publish(cmd)
        mission = self.strategy.mission
        self.pose_pub.publish(Float32MultiArray(data=[float(v) for v in (*mission.pose, mission.phi)]))
        self._log_parking(mission)
        if mission.brake:                                # the front recover's hard brake
            self.estop.publish(Bool(data=True))
        for entry in mission.events[self.events_logged:]:
            if isinstance(entry, dict) and entry.get('event') == 'steer_fault':
                self.get_logger().warn('[%6.2f s] STEER FAULT: full lock %+.1f deg for %.2f s, gyro %+.2f rad/s of %+.2f '
                                       'expected -> stop and centre (fault %d)' % (
                                           self._t() or 0.0, entry['steer'], entry['lock_s'], entry['gyro'],
                                           entry['expected'], entry['count']))
                self.rec.event('steer_fault', self._t(), **entry)
                continue
            self.get_logger().info('recover: %s' % entry)
            self.rec.event('recover', self._t(), detail=str(entry))
        self.events_logged = len(mission.events)
        if mission.lot_exit is not None:
            if mission.lot_exit.brake:
                self.estop.publish(Bool(data=True))
            for entry in mission.lot_exit.log[self.lot_logged:]:
                self.get_logger().info('lot exit: %s' % entry)
                self.rec.event('lot_exit', self._t(), detail=str(entry))
            self.lot_logged = len(mission.lot_exit.log)
        self._log_progress(mission)
        if self.strategy.camera_lost != self.camera_lost_logged:
            self.camera_lost_logged = self.strategy.camera_lost
            if self.camera_lost_logged:
                self.get_logger().warn('[%6.2f s] CAMERA LOST: no frame for %.1f s, driving on with the remembered colours'
                                       % (self._t() or 0.0, self.now() - self.strategy.signs_stamp))
            else:
                self.get_logger().info('[%6.2f s] camera back' % (self._t() or 0.0))
            self.rec.event('camera_lost' if self.camera_lost_logged else 'camera_back', self._t())
        if self.strategy.label != self.state_sent:
            self.state_sent = self.strategy.label
            stamp = '[%6.2f s] ' % self._t() if self.t_start is not None else ''
            if self.state_sent == 'WAIT_SENSOR':
                ages = self.strategy.stale(self.now())
                self.get_logger().warn('%sstate WAIT_SENSOR: stale %s' % (stamp, ', '.join(
                    '%s %.2f s' % (name, age) for name, age in sorted(ages.items()))))
                self.rec.event('wait_sensor', self._t(), stale=ages)
            else:
                self.get_logger().info('%sstate %s' % (stamp, self.state_sent))
                self.rec.event('state', self._t(), state=self.state_sent)
            if self.state_sent == 'DONE' and self.t_start is not None:
                self.get_logger().info('\033[1;32mDONE: %.2f s from Start\033[0m' % self._t())
                self.rec.event('done', self._t(), progress=mission.phi)
                self._log_round('DONE')
            elif self.state_sent == 'FAULT':
                self.rec.event('fault', self._t(), progress=mission.phi)
                self._log_round('FAULT')
            self.state.publish(String(data=self.state_sent))
        self._record(mission, speed, steer)

    def _t(self):
        """Seconds since Start on the node's clock (None before Start)."""
        return None if self.t_start is None else self.now() - self.t_start

    def _log_progress(self, mission):
        """The terminal summary (tools/car-run.sh keeps it), as the Open v5: every section (straight or corner) with
        its time, each lap, each sign when it is first seen and when its colour is known, each sign passed (side and
        whether it is the side its colour asks for). Progress units: 8 a lap, corner k = units 2k-1..2k."""
        if self.t_start is None or mission.state not in ('RUN', 'PARK', 'STOP'):
            return
        t, phi = self._t(), mission.phi
        unit = int(math.floor(phi))
        if self.section is None:
            self.section, self.section_start = unit, t
        while self.section < unit and self.section < 24:
            kind = 'corner %d' % ((self.section + 1) // 2) if self.section % 2 else 'straight %d' % (self.section // 2 % 4 + 1)
            took = t - self.section_start
            self.sections.append((self.section, kind, took))
            self.section += 1
            self.section_start = t
            nxt = 'corner %d' % ((self.section + 1) // 2) if self.section % 2 else 'straight %d' % (self.section // 2 % 4 + 1)
            self.get_logger().info('[%6.2f s] %s (lap %d); %s took %.2f s' % (
                t, nxt, self.section // 8 + 1, kind, took))
            self.rec.event('section', t, section=self.section, section_name=nxt, previous=kind, previous_s=took, progress=phi)
            if self.section % 2 and self.corners_logged < 12:
                self.corners_logged += 1
                self.rec.event('corner', t, corner=self.corners_logged, progress=phi)
        laps = min(3, int(phi // UNITS_PER_LAP))
        while self.laps_logged < laps:
            self.laps_logged += 1
            lap_time, self.lap_start = t - self.lap_start, t
            self.laps.append(lap_time)
            self.get_logger().info('\033[1;36mLAP %d: %.2f s (total %.2f s)\033[0m' % (self.laps_logged, lap_time, t))
            self.rec.event('lap', t, lap=self.laps_logged, lap_time=lap_time, progress=phi)
        self._log_signs(mission, t, phi)

    def _log_signs(self, mission, t, phi):
        for seat, (lateral, votes, hits) in mission._seats.items():
            if hits < SEAT_HITS:
                continue
            colour = max(votes, key=votes.get) if votes else None     # in the mission's frame (CCW)
            shown = colour if mission.selected_direction == 'ccw' or colour is None else SWAP.get(colour, colour)
            where = 'straight %d seat %.1f, %s row' % (seat // 4 % 4 + 1, seat / 2.0 % 2.0,
                                                      'inner' if lateral > 0 else 'outer')
            if seat not in self.signs_known or (colour is not None and self.signs_known[seat] != colour):
                first = seat not in self.signs_known
                self.signs_known[seat] = colour
                self.get_logger().info('[%6.2f s] sign %s: %s, %s' % (
                    t, 'seen' if first else 'colour', where, shown or 'colour unknown'))
                self.rec.event('sign_seen' if first else 'sign_colour', t, seat=seat, lateral=lateral,
                               colour=shown, votes=dict(votes), progress=phi)
            lap = int(phi // UNITS_PER_LAP)
            at = lap * UNITS_PER_LAP + seat / 2.0
            if (lap, seat) in self.signs_passed or not at <= phi < at + 0.3:
                continue
            self.signs_passed.add((lap, seat))
            x, y, _ = mission.pose
            px, py, _ = scene.lane_pose(at)
            nx, ny = scene.lane_normal(at)
            car = (x - px) * nx + (y - py) * ny          # the car's lateral position on the lane at the sign
            wanted = mission._side_offset(lateral, colour) if colour else None
            side = 'inner' if car > lateral else 'outer'
            ok = '' if wanted is None else (' OK' if (car - lateral) * (wanted - lateral) > 0 else ' WRONG SIDE')
            self.get_logger().info('[%6.2f s] sign passed: %s, %s, car on its %s side (%.2f m off the sign)%s' % (
                t, where, shown or 'colour unknown', side, abs(car - lateral), ok))
            self.rec.event('sign_passed', t, seat=seat, lap=lap + 1, colour=shown, lateral=lateral,
                           car_lateral=round(car, 3), side=side, ok=None if wanted is None else ok == ' OK')

    def _log_round(self, how):
        """One line with the whole round: total time from Start, the laps and the slowest section."""
        if self.round_logged or self.t_start is None:
            return
        self.round_logged = True
        t = self._t()
        slow = max(self.sections, key=lambda s: s[2]) if self.sections else None
        line = 'ROUND %s: %.2f s from Start, laps %s' % (how, t, ' / '.join('%.2f' % v for v in self.laps) or '-')
        if slow:
            line += ', slowest section %d (%s) %.2f s' % (slow[0] + 1, slow[1], slow[2])
        self.get_logger().info('\033[1;32m%s\033[0m' % line if how == 'DONE' else line)
        self.rec.event('round', t, end=how, laps=self.laps, sections=[list(s) for s in self.sections])

    def _record(self, mission, speed, steer):
        if not self.rec.on:
            return
        now, scan = self.now(), self.scan_msg
        x, y, th = mission.pose
        pillar = mission.pillar or (None, None, None)
        parking = mission.parking
        self.rec.row(
            wall_time=time.time(), t=self._t(), state=self.strategy.label, progress=round(mission.phi, 4),
            lap=int(mission.phi // UNITS_PER_LAP) + 1, corner=self.corners_logged,
            x=round(x, 4), y=round(y, 4), th=round(th, 4), yaw=round(self.strategy.yaw, 4),
            gyro=round(self.strategy.gyro, 4), gyro_bias=round(mission.gyro_bias, 5),
            travelled=round(self.strategy.travelled, 5), v_cmd=round(speed, 4), steer_cmd=round(steer, 4),
            front=round(sector_min(scan, 0.0), 3), left=round(sector_min(scan, math.pi / 2), 3),
            right=round(sector_min(scan, -math.pi / 2), 3), rear_mm=self.rear_mm,
            pillar_lateral=pillar[0], pillar_ahead=None if pillar[1] is None else round(pillar[1] - mission.phi, 3),
            pillar_colour=pillar[2], offset=round(mission._offset_at(mission.phi), 4),
            park_state=None if parking is None else parking.state,
            scan_age=round(now - self.strategy.scan_stamp, 3), imu_age=round(now - self.strategy._imu_stamp, 3),
            camera_age=round(now - self.strategy.signs_stamp, 3), signs_seen=len(self.strategy.signs))


    def _log_parking(self, mission):
        """The parking's own steps and events: each sub-state (PARK_APPROACH, PARK_MOVE, ...) with the pose, the
        log of park.py / park_fixed.py (start pose, LiDAR correction, end) and the verdict, PARKED or PARK_FAIL and
        why (on the car on 8 nov 2026 a DONE could be either)."""
        parking = mission.parking
        if parking is None:
            return
        # The fixed park's pose against the lot (with its LiDAR correction), else the mission's.
        own = parking._corrected(*parking.raw) if getattr(parking, 'raw', None) else mission.pose
        pose = ' '.join('%.3f' % v for v in own)
        if parking.state != self.park_state:
            self.park_state = parking.state
            move = getattr(parking, 'current', None)
            what = '' if move is None else ' move steer %.3f dir %+.0f target %s' % (move[0], move[1], move[3][0])
            self.get_logger().info('park %s at %s%s' % (parking.state, pose, what))
            self.rec.event('park_state', self._t(), state=parking.state, pose=list(own))
        for event in parking.log:
            self.get_logger().info('park event %s' % {k: ([round(float(x), 4) for x in v] if isinstance(v, (list, tuple))
                                                          else v) for k, v in event.items()})
            if event.get('event') == 'park_end':
                inside = parking.lot.parked(*own, inside=0.0, parallel=math.radians(90.0))
                if event.get('partial') and not event.get('parked'):
                    why = 'partial park: entry and straight into the lot done (no straightening)'
                    self.get_logger().info('park verdict PARTIAL: %s, %d moves' % (why, event.get('moves', 0)))
                    self.rec.event('park_verdict', self._t(), parked='partial', why=why,
                                   moves=event.get('moves', 0), pose=list(own))
                    continue
                why = 'parked: inside and parallel (own pose)' if event.get('parked') else (
                    'not parallel: heading %.1f deg' % math.degrees(own[2]) if inside
                    else 'not inside the lot (own pose)')
                self.get_logger().info('park verdict %s: %s, %d moves' % (
                    'PARKED' if event.get('parked') else 'PARK_FAIL', why, event.get('moves', 0)))
                self.rec.event('park_verdict', self._t(), parked=bool(event.get('parked')), why=why,
                               moves=event.get('moves', 0), pose=list(own))
            else:
                self.rec.event('park', self._t(), detail={k: (list(v) if isinstance(v, tuple) else v)
                                                          for k, v in event.items()})
        parking.log.clear()


def main(args=None):
    rclpy.init(args=args)
    node = Drive()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        node._log_round('STOPPED')
        node.rec.event('stop', node._t())
        node.rec.close()
        if rclpy.ok():
            node.cmd.publish(Twist())
            rclpy.shutdown()
