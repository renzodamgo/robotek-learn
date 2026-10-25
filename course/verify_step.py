#!/usr/bin/env python3
"""Check a step of the course in Gazebo, with no window: does the car do what that step promises?

    python3 course/verify_step.py N [--param nombre:=valor ...] [--seconds S] [--timeout 240] [--keep-logs] [--compare]

Run it in a terminal where ROS and the course workspace are sourced (and the workspace with YOUR ``my_open``, built
for step N, on top of it). It starts the simulator (``ros2 launch robotek_course sim.launch.py gui:=false``), starts
your node (``ros2 run my_open drive``), publishes /start when the sensors are alive, records the TRUE pose of the car
(/odom_gt: your node never sees it, only this checker does), stops everything and prints PASS or FAIL with the
numbers. Exit code: 0 PASS, 1 FAIL, 2 the checker could not run (for example another Gazebo is already running).

    step 1  a circle with the wheels fixed: its radius agrees with model.turn_radius
    step 2  a straight line holding heading 0: the true heading after 10 s, with the gyroscope calibration
    step 3  the four numbers of /scene agree with the real geometry; the car stops before the wall
    step 4  along a long island wall: the gap settles at 0.25 m (+-0.06), no contact, the car stops before the wall ahead
    step 5  after the first corner the heading is 90 degrees (counter-clockwise)
    step 6  round and round the island: at least 8 corners in 40 s without touching a wall

Not every step runs in the standard field (``--world field|empty|lane`` chooses):
  * steps 1 and 2 run in an EMPTY field (the walls removed): a circle does not fit in a 1 m corridor and 10 s of
    straight driving is longer than the field;
  * step 4 runs in a field whose island wall is 9 m long: the lane law closes a gap error with a time constant
    of about 5 s, and the standard island is only 1 m long (``--lane-world FILE`` writes that world).
"""
import argparse
import math
import os
import re
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

try:
    import numpy as np
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import Bool, Float32MultiArray
    from visualization_msgs.msg import MarkerArray
except ImportError as error:          # pragma: no cover
    sys.exit('verify_step: falta el entorno de ROS (%s). Carga ROS y el workspace antes de ejecutarlo.' % error)

from robotek_course.minisim import Field, wrap
from robotek_evaluation.open_field_geometry import field_geometry

WORLD = 'robotek_open_field.sdf'
SETTLE_S = 1.0               # sim seconds with every sensor alive before Start
CALIBRATE_S = 0.6            # the stand-still of steps 2 to 6 (the drive files wait this long after Start)
MIN_SCENES = 15
BRAKING_FRONT = 0.8          # step 3: m. Closer to the wall than this the car is braking: the scene is not compared
DEFAULT_STEER = 0.25         # the default of the ``steer`` parameter in drive_step1.py

# Tolerances (what was measured is in course/snapshots/README.md).
CIRCLE_TOLERANCE = 0.04      # step 1: relative error of the radius
YAW_TOLERANCE = 0.03         # step 2: rad
SCENE_TOLERANCE = 0.05       # step 3: m
END_TOLERANCE = 0.10         # step 3: m (island_end, when both are visible)
HEADING_TOLERANCE = 0.12     # step 5: rad
MIN_CORNERS = 8              # step 6
STEP_SECONDS = {1: 14.0, 2: CALIBRATE_S + 10.0, 3: 8.0, 4: 40.0, 5: 20.0, 6: 40.0}   # sim seconds after Start
WORLD_OF_STEP = {1: 'empty', 2: 'empty', 3: 'field', 4: 'lane', 5: 'field', 6: 'field'}
STOPS_BY_ITSELF = {3, 4, 5}                                                       # the run ends when the car stops


# ---------------------------------------------------------------------------------------------- the world
def _without_mat(text):
    """The world without the game mat. The mat is only 3 x 3 m: a car that drives off its edge pitches and the
    LiDAR sees the floor as a wall for a moment, which a bigger field must not do."""
    return re.sub(r'<include>\s*<uri>model://robotek_game_mat</uri>.*?</include>', '', text, flags=re.S)


def empty_world(destination):
    """Write the Open field world without the walls (``game_field``) and the mat to ``destination``; returns the path."""
    text = open(field_world()).read()
    start = text.index('<model name="game_field">')
    end = text.rindex('</model>', start) + len('</model>')
    with open(destination, 'w') as out:
        out.write(_without_mat(text[:start] + text[end:]))
    return destination


def field_world():
    return os.path.join(get_package_share_directory('robotek_gazebo'), 'worlds', WORLD)


def _set_box(text, name, pose, size):
    """Change the pose and the size of the box called ``name`` (its visual and its collision)."""
    pattern = r'(name="%s(?:_col)?">\s*<pose>)[^<]*(</pose>\s*<geometry><box><size>)[^<]*(</size>)' % name
    new, count = re.subn(pattern, lambda m: '%s%s%s%s%s' % (m.group(1), pose, m.group(2), size, m.group(3)), text)
    if count != 2:
        raise ValueError('wall %s not found in the world' % name)
    return new


def lane_world(destination):
    """The Open field with a 9 m island wall along the south corridor, so the lane law has room to settle.

    Same corridor (1 m wide, island face at y = -0.5, outer wall at y = -1.5), same height and colours, no mat. The island
    runs from x = -0.5 to 9.0 and the east wall is at x = 10.0 (1 m beyond the island's end, like the real field).
    """
    text = _without_mat(open(field_world()).read())
    for name, pose, size in (
            ('wall_ext_east', '10.02 0 0.06 0 0 0', '0.04 3.08 0.10'),
            ('wall_ext_north', '4.24 1.52 0.06 0 0 0', '11.64 0.04 0.10'),
            ('wall_ext_south', '4.24 -1.52 0.06 0 0 0', '11.64 0.04 0.10'),
            ('wall_int_south', '4.25 -0.48 0.06 0 0 0', '9.50 0.04 0.10'),
            ('wall_int_north', '4.25 0.48 0.06 0 0 0', '9.50 0.04 0.10'),
            ('wall_int_east', '8.98 0 0.06 0 0 0', '0.04 1.00 0.10')):
        text = _set_box(text, name, pose, size)
    with open(destination, 'w') as out:
        out.write(text)
    return destination


def walls_of(world):
    """[(centre x, centre y, size x, size y)] of every wall box of the world's ``game_field``, read from the SDF."""
    link = ET.parse(world).find(".//model[@name='game_field']/link[@name='field']")
    walls = []
    for node in link.findall('collision'):
        pose = [float(v) for v in node.findtext('pose').split()]
        size = [float(v) for v in node.findtext('geometry/box/size').split()]
        walls.append((pose[0], pose[1], size[0], size[1]))
    return tuple(walls)


# ---------------------------------------------------------------------------------------- processes
def gz_sim_pids():
    """pids of every ``gz sim`` on the machine (by command line: it must START with gz sim)."""
    out = subprocess.run(['ps', '-axo', 'pid=,command='], capture_output=True, text=True).stdout
    found = []
    for line in out.splitlines():
        pid, _, command = line.strip().partition(' ')
        if re.match(r'^(\S*/)?gz sim\b', command.strip()):
            found.append(int(pid))
    return found


class Children:
    """The processes we start, each in its own process group, and a clean way to end all of them."""

    def __init__(self):
        self.procs = []
        self.groups = set()

    def start(self, command, log):
        # Gazebo's transport does not know ROS_DOMAIN_ID: a private partition keeps another simulator on this machine
        # (other terminal, other user session) from answering our /cmd_vel and sensor topics.
        env = dict(os.environ, GZ_PARTITION='verify_step_%d' % os.getpid())
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                start_new_session=True, env=env)
        self.procs.append(proc)
        self.groups.add(os.getpgid(proc.pid))
        return proc

    def _alive(self):
        out = subprocess.run(['ps', '-axo', 'pid=,pgid='], capture_output=True, text=True).stdout
        return [int(p) for p, g in (line.split() for line in out.splitlines() if line.strip())
                if int(g) in self.groups and int(p) != os.getpid()]

    def _signal(self, sig):
        for group in self.groups:
            try:
                os.killpg(group, sig)
            except (ProcessLookupError, PermissionError):
                pass

    def stop(self):
        self._signal(signal.SIGINT)
        deadline = time.time() + 8.0
        while time.time() < deadline and self._alive():
            time.sleep(0.25)
        for sig, wait in ((signal.SIGTERM, 3.0), (signal.SIGKILL, 2.0)):
            if not self._alive():
                break
            self._signal(sig)
            deadline = time.time() + wait
            while time.time() < deadline and self._alive():
                time.sleep(0.25)
        for proc in self.procs:
            try:
                proc.wait(timeout=1)
            except subprocess.TimeoutExpired:
                pass
        return self._alive()


# ------------------------------------------------------------------------------------------- the monitor
def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Monitor(Node):
    """Listens to the true pose and the sensors, publishes /start, and keeps what the checks need."""

    def __init__(self):
        super().__init__('verify_step', parameter_overrides=[Parameter(
            'use_sim_time', Parameter.Type.BOOL, True)])
        self.alive = {}                 # sensor -> sim time of its first message
        self.poses = []                 # (sim time, x, y, yaw): the TRUE pose
        self.scans = []                 # (stamp, perceive(scan) as a tuple): the scene the node should publish
        self.scenes = []                # the numbers the node published on /scene
        self.markers = 0
        self.start_time = None
        self.start_pub = self.create_publisher(Bool, 'start', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, '/odom_gt', self.on_pose, 50)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', lambda m: self.mark('imu'), qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, '/scene', lambda m: self.scenes.append(tuple(m.data)), 50)
        self.create_subscription(MarkerArray, '/scene_markers', self.on_markers, 10)
        self.perceive = None
        try:
            from my_open.perception import perceive
            self.perceive = perceive
        except ImportError:
            pass
        self.last_stamp = None

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def mark(self, name):
        self.alive.setdefault(name, self.now())

    def on_pose(self, msg):
        self.mark('pose')
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        self.poses.append((t, p.x, p.y, yaw_of(msg.pose.pose.orientation)))

    def on_scan(self, msg):
        self.mark('scan')
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.perceive and stamp != self.last_stamp and self.start_time is not None:
            self.last_stamp = stamp
            scene = self.perceive(msg, forward_yaw=0.0, sign=1)
            self.scans.append((stamp, (scene.left, scene.right, scene.front_axis, scene.island_end)))

    def on_markers(self, msg):
        if len(msg.markers) == 4:
            self.markers += 1

    def ready(self):
        return (len(self.alive) == 3 and self.now() - max(self.alive.values()) >= SETTLE_S
                and self.count_subscribers('start') > 0)

    def start(self):
        self.start_time = self.now()
        self.start_pub.publish(Bool(data=True))

    def pose_at(self, t):
        """Linear interpolation of the true pose at sim time ``t`` (None outside the record)."""
        times = [p[0] for p in self.poses]
        i = int(np.searchsorted(times, t))
        if i == 0 or i >= len(times):
            return None
        (t0, x0, y0, a0), (t1, x1, y1, a1) = self.poses[i - 1], self.poses[i]
        k = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return x0 + k * (x1 - x0), y0 + k * (y1 - y0), wrap(a0 + k * wrap(a1 - a0))


# -------------------------------------------------------------------------------------------- geometry
def clearance(field, x, y, yaw):
    """Smallest distance (m) from the car's outline to a wall; 0 when they touch."""
    from robotek_course.minisim import OUTLINE
    front, rear, half = OUTLINE
    c, s = math.cos(yaw), math.sin(yaw)
    corners = [(front, half), (front, -half), (-rear, -half), (-rear, half)]
    points = []
    for (ax, ay), (bx, by) in zip(corners, corners[1:] + corners[:1]):
        for k in range(12):
            points.append((ax + (bx - ax) * k / 12, ay + (by - ay) * k / 12))
    best = math.inf
    for lx, ly in points:
        px, py = x + lx * c - ly * s, y + lx * s + ly * c
        for cx, cy, sx, sy in field.walls:
            dx, dy = max(abs(px - cx) - sx / 2, 0.0), max(abs(py - cy) - sy / 2, 0.0)
            best = min(best, math.hypot(dx, dy))
    return best


def contacts(field, poses):
    """(times of contact, smallest clearance) over the record."""
    touched, nearest = [], math.inf
    for t, x, y, yaw in poses[::2]:
        if field.touches(x, y, yaw):
            touched.append(t)
        nearest = min(nearest, clearance(field, x, y, yaw))
    return touched, nearest


def fit_circle(points):
    """Least-squares circle through (x, y) points: returns (cx, cy, radius, rms residual)."""
    a = np.array(points)
    design = np.column_stack([2 * a[:, 0], 2 * a[:, 1], np.ones(len(a))])
    (cx, cy, c), *_ = np.linalg.lstsq(design, (a ** 2).sum(axis=1), rcond=None)
    radius = math.sqrt(c + cx * cx + cy * cy)
    rms = float(np.sqrt(np.mean((np.hypot(a[:, 0] - cx, a[:, 1] - cy) - radius) ** 2)))
    return cx, cy, radius, rms


def unwrap(angles):
    return np.unwrap(np.array(angles))


def stopped(poses, window=1.0):
    """True when the car has moved and has not moved more than 1 cm in the last ``window`` sim seconds."""
    if len(poses) < 10 or math.hypot(poses[-1][1] - poses[0][1], poses[-1][2] - poses[0][2]) < 0.1:
        return False
    t_end = poses[-1][0]
    recent = [p for p in poses if p[0] >= t_end - window]
    return recent[0][0] <= t_end - window + 0.2 and math.hypot(recent[-1][1] - recent[0][1], recent[-1][2] - recent[0][2]) < 0.01


# ------------------------------------------------------------------------------------------ the checks
class Report:
    def __init__(self):
        self.lines, self.failed = [], False

    def info(self, text):
        self.lines.append('  ' + text)

    def check(self, ok, text):
        self.lines.append('  [%s] %s' % ('OK' if ok else 'FALLA', text))
        self.failed = self.failed or not ok

    def numbers(self, **kv):
        self.info(', '.join('%s=%s' % (k, v) for k, v in kv.items()))


def run_poses(monitor):
    return [p for p in monitor.poses if p[0] >= monitor.start_time]


def check_1(monitor, params, report, steer):
    from my_open.model import turn_radius
    poses = [p for p in run_poses(monitor) if p[0] >= monitor.start_time + 1.0]
    if len(poses) < 50:
        return report.check(False, 'hay muy pocas poses (%d): el coche no se movio' % len(poses))
    cx, cy, radius, rms = fit_circle([(p[1], p[2]) for p in poses])
    expected = turn_radius(steer)
    error = (radius - expected) / expected
    angle = abs(float(unwrap([p[3] for p in poses])[-1] - unwrap([p[3] for p in poses])[0]))
    report.info('volante %.3f rad: el modelo espera un radio de %.3f m' % (steer, expected))
    report.info('el ajuste de un circulo a la trayectoria da %.3f m (error del ajuste %.1f mm, %.1f vueltas)'
                % (radius, rms * 1000, angle / (2 * math.pi)))
    report.check(abs(error) <= CIRCLE_TOLERANCE, 'radio medido %.3f m, esperado %.3f m: diferencia %+.1f %% (tolerancia %.0f %%)'
                 % (radius, expected, 100 * error, 100 * CIRCLE_TOLERANCE))


def check_2(monitor, params, report):
    poses = run_poses(monitor)
    if not poses:
        return report.check(False, 'no hay poses')
    calibrate = str(params.get('calibrate', 'true')).lower() != 'false'
    yaw = poses[-1][3]
    drift = math.degrees(yaw)
    report.info('rumbo verdadero al final (%.1f s tras Start, %.2f m recorridos): %+.4f rad (%+.2f grados)'
                % (poses[-1][0] - monitor.start_time, math.hypot(poses[-1][1] - poses[0][1], poses[-1][2] - poses[0][2]),
                   yaw, drift))
    if calibrate:
        report.check(abs(yaw) < YAW_TOLERANCE, 'con calibracion el error de rumbo es %.4f rad (limite %.2f rad)' % (abs(yaw), YAW_TOLERANCE))
    else:
        report.info('sin calibracion (calibrate:=false): la deriva es %.4f rad. Esto no es una condicion de PASS.' % abs(yaw))


def lidar_xy(x, y, yaw):
    from robotek_course.minisim import LIDAR_X
    return x + LIDAR_X * math.cos(yaw), y + LIDAR_X * math.sin(yaw)


def check_3(monitor, params, report, bounds, field):
    ideal = Field(walls=field.walls, seed=0)
    pairs, worst = 0, [0.0, 0.0, 0.0]
    end_errors = []
    for numbers in monitor.scenes:
        match = next((s for s in monitor.scans if all((a == b) or (math.isinf(a) and math.isinf(b)) or abs(a - b) < 1e-4
                                                      for a, b in zip(numbers, s[1]))), None)
        if match is None:
            continue
        # The stamp of a scan and the time stamp of the pose are not exact: accept the truth at the scan time +-0.06 s
        # (+-0.15 s for left and island_end, which jump when the island wall ends beside the car)
        shifts = (-0.15, -0.1, -0.06, 0.0, 0.06, 0.1, 0.15)
        expected = {d: _ideal_scene(ideal, pose) for d in shifts if (pose := monitor.pose_at(match[0] + d))}
        if len(expected) < len(shifts) or expected[0.0][2] < BRAKING_FRONT:
            continue      # after the stop order the car pitches forward and the LiDAR sees the floor as a wall
        pairs += 1
        if os.environ.get('VERIFY_DEBUG'):
            print('t=%.2f scene=%s expected=%s' % (match[0] - monitor.start_time, ['%.3f' % v for v in numbers],
                                                  ['%.3f' % v for v in expected[0.0]]))
        near = [e for d, e in expected.items() if abs(d) <= 0.06]
        lefts = [e[0] for e in expected.values()]
        if max(lefts) - min(lefts) <= SCENE_TOLERANCE:      # not while the island's end passes beside the car: the median jumps
            worst[0] = max(worst[0], min(_diff(numbers[0], v) for v in lefts))
        worst[1] = max(worst[1], min(_diff(numbers[1], e[1]) for e in near))
        worst[2] = max(worst[2], min(_diff(numbers[2], e[2]) for e in near))
        ends = [e[3] for e in expected.values()]
        if all(math.isfinite(v) for v in ends) and math.isfinite(numbers[3]):       # only where the end is clearly in view
            end_errors.append(min(abs(numbers[3] - e[3]) for e in near))
    poses = run_poses(monitor)
    report.info('/scene: %d mensajes, %d emparejados con un escaneo y la pose verdadera; /scene_markers: %d mensajes con 4 marcadores'
                % (len(monitor.scenes), pairs, monitor.markers))
    report.check(len(monitor.scenes) >= MIN_SCENES and pairs >= MIN_SCENES, 'el nodo publica /scene (al menos %d mensajes)' % MIN_SCENES)
    report.check(monitor.markers >= MIN_SCENES, 'el nodo publica /scene_markers con 4 marcadores por mensaje')
    for name, error in zip(('left', 'right', 'front_axis'), worst):
        report.check(error <= SCENE_TOLERANCE, '%s: peor diferencia con la geometria %.3f m (limite %.2f m)' % (name, error, SCENE_TOLERANCE))
    if end_errors:
        report.check(max(end_errors) <= END_TOLERANCE, 'island_end: peor diferencia %.3f m en %d mensajes (limite %.2f m)'
                     % (max(end_errors), len(end_errors), END_TOLERANCE))
    else:
        report.info('island_end: nunca visible a la vez en el nodo y en la geometria')
    if poses:
        x, y, yaw = poses[-1][1:]
        lx, ly = lidar_xy(x, y, yaw)
        front = bounds['outer']['xmax'] - lx
        report.check(0.10 <= front <= 0.50, 'el coche se paro con la pared a %.2f m del LiDAR (entre 0.10 y 0.50 m)' % front)
        report.check(stopped(poses), 'el coche esta parado al final')
    touched, nearest = contacts(field, poses)
    report.check(not touched, 'no toca ninguna pared (distancia minima %.3f m)' % nearest)


def _ideal_scene(field, pose):
    from my_open.perception import perceive
    scene = perceive(field.scan(*pose, noise=False, dropouts=False), forward_yaw=0.0, sign=1)
    return scene.left, scene.right, scene.front_axis, scene.island_end


def _diff(a, b):
    if math.isinf(a) and math.isinf(b):
        return 0.0
    return abs(a - b) if math.isfinite(a) and math.isfinite(b) else math.inf


def check_4(monitor, params, report, bounds, field, kind):
    from my_open.lane import lateral_bias
    poses = run_poses(monitor)
    if not poses:
        return report.check(False, 'no hay poses')
    ymin = bounds['inner']['ymin']
    gaps = [(ymin - y) * math.cos(yaw) for _, _, y, yaw in poses]
    # The order to stop comes when the wall ahead is at 1.19 m (from the LiDAR); look at the gap right there.
    order = next((i for i, (_, x, y, yaw) in enumerate(poses) if bounds['outer']['xmax'] - lidar_xy(x, y, yaw)[0] <= 1.19),
                 len(poses) - 1)
    x, y, yaw = poses[-1][1:]
    front = bounds['outer']['xmax'] - lidar_xy(x, y, yaw)[0]
    touched, nearest = contacts(field, poses)
    report.info('distancia a la isla: %.3f m al empezar, %.3f m al dar la orden de parar (a los %.1f m), %.3f m (minimo %.3f) al terminar'
                % (gaps[0], gaps[order], poses[order][1] - poses[0][1], gaps[-1], min(gaps)))
    report.info('rumbo verdadero al terminar %+.3f rad; parado a %.2f m de la pared de enfrente (medido desde el LiDAR)' % (yaw, front))
    report.check(stopped(poses), 'el coche se paro')
    report.check(0.05 <= front <= 1.19, 'parado antes de la pared de enfrente (a %.2f m; la orden sale a 1.19 m y el coche frena despues)' % front)
    if kind == 'lane':
        report.check(0.20 <= gaps[order] <= 0.31, 'distancia final a la isla %.3f m dentro de [0.20, 0.31] (la ley pide 0.25 m)' % gaps[order])
        report.check(min(gaps[order // 2:]) >= 0.20, 'nunca mas cerca de la isla que 0.20 m en la segunda mitad (minimo %.3f m)' % min(gaps[order // 2:]))
    else:
        low, high = lateral_bias(gaps[-1]) - 0.03, lateral_bias(gaps[0]) + 0.08
        report.check(gaps[order] < gaps[0], 'se acerca a la isla: %.3f -> %.3f m' % (gaps[0], gaps[order]))
        report.check(low <= yaw <= high, 'rumbo %+.3f rad dentro de lo que pide la ley (entre %+.3f y %+.3f)' % (yaw, low, high))
    report.check(not touched, 'no toca ninguna pared (distancia minima %.3f m)' % nearest)


def check_5(monitor, params, report, bounds, field):
    poses = run_poses(monitor)
    if not poses:
        return report.check(False, 'no hay poses')
    x, y, yaw = poses[-1][1:]
    touched, nearest = contacts(field, poses)
    inner = bounds['inner']
    error = wrap(yaw - math.pi / 2)
    report.info('posicion final (%.2f, %.2f), rumbo verdadero %+.3f rad (pi/2 = 1.571)' % (x, y, yaw))
    report.check(abs(error) <= HEADING_TOLERANCE, 'tras la primera esquina el rumbo es pi/2 %+.3f rad (limite +-%.2f)' % (error, HEADING_TOLERANCE))
    report.check(x > inner['xmax'] and y > inner['ymin'], 'esta en el pasillo del este, junto a la isla (x %.2f > %.2f)' % (x, inner['xmax']))
    report.check(stopped(poses), 'el coche se paro ante la pared siguiente')
    report.check(not touched, 'no toca ninguna pared (distancia minima %.3f m)' % nearest)


def check_6(monitor, params, report, bounds, field):
    poses = run_poses(monitor)
    if len(poses) < 100:
        return report.check(False, 'hay muy pocas poses (%d)' % len(poses))
    angle = unwrap([p[3] for p in poses])
    total = float(angle[-1] - angle[0])
    corners = int((total + 0.3) // (math.pi / 2))
    touched, nearest = contacts(field, poses)
    distance = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(poses, poses[1:]))
    report.info('en %.0f s: giro total %.2f rad = %d esquinas (%.2f vueltas), %.1f m recorridos'
                % (poses[-1][0] - monitor.start_time, total, corners, total / (2 * math.pi), distance))
    report.check(corners >= MIN_CORNERS, 'da al menos %d esquinas (da %d)' % (MIN_CORNERS, corners))
    report.check(not touched, 'no toca ninguna pared (distancia minima %.3f m%s)'
                 % (nearest, '' if not touched else '; primer contacto a los %.1f s' % (touched[0] - monitor.start_time)))


# ----------------------------------------------------------------------------------------------- main
def run(step, params, args):
    logs = args.log_dir or tempfile.mkdtemp(prefix='verify_step%d_' % step)
    os.makedirs(logs, exist_ok=True)
    kind = args.world or WORLD_OF_STEP[step]
    world = {'empty': lambda: empty_world(os.path.join(logs, 'empty_field.sdf')),
             'lane': lambda: lane_world(os.path.join(logs, 'lane_field.sdf')),
             'field': field_world}[kind]()
    bounds = None if kind == 'empty' else field_geometry(world)
    field = None if kind == 'empty' else Field(walls=walls_of(world))
    seconds = args.seconds or STEP_SECONDS[step]
    values = dict(p.split(':=', 1) for p in params)
    children = Children()
    report = Report()
    result = 'FAIL'
    started_wall = time.time()
    rclpy.init(args=[])
    monitor = Monitor()
    try:
        children.start(['ros2', 'launch', 'robotek_course', 'sim.launch.py', 'gui:=false', 'world:=' + world],
                       open(os.path.join(logs, 'sim.log'), 'w'))
        drive = ['ros2', 'run', 'my_open', 'drive', '--ros-args', '-p', 'use_sim_time:=true']
        for param in params:
            drive += ['-p', param]
        children.start(drive, open(os.path.join(logs, 'drive.log'), 'w'))
        phase = 'esperando a los sensores y al nodo drive'
        ended = None
        while time.time() - started_wall < args.timeout:
            rclpy.spin_once(monitor, timeout_sec=0.05)
            if monitor.start_time is None:
                if monitor.ready():
                    monitor.start()
                    phase = 'en marcha'
                continue
            elapsed = monitor.now() - monitor.start_time
            poses = run_poses(monitor)
            if elapsed >= seconds:
                ended = 'duracion'
                break
            if step in STOPS_BY_ITSELF and elapsed > CALIBRATE_S + 2.0 and stopped(poses):
                for _ in range(20):                 # a second more, to collect the last scans
                    rclpy.spin_once(monitor, timeout_sec=0.05)
                ended = 'parado'
                break
        else:
            report.check(False, 'tiempo de espera agotado (%d s de reloj) mientras: %s; mira %s' % (args.timeout, phase, logs))
        if ended:
            poses = run_poses(monitor)
            report.info('simulacion: %.1f s tras Start (%s), %d poses, reloj de pared %.0f s'
                        % (monitor.now() - monitor.start_time, ended, len(poses), time.time() - started_wall))
            if step == 1:
                check_1(monitor, values, report, float(values.get('steer', DEFAULT_STEER)))
            elif step == 2:
                check_2(monitor, values, report)
            elif step == 3:
                check_3(monitor, values, report, bounds, field)
            elif step == 4:
                check_4(monitor, values, report, bounds, field, kind)
            elif step == 5:
                check_5(monitor, values, report, bounds, field)
            else:
                check_6(monitor, values, report, bounds, field)
        result = 'FAIL' if report.failed else 'PASS'
    except KeyboardInterrupt:
        report.check(False, 'interrumpido')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()
        left = children.stop()
        if left:
            print('verify_step: quedaron procesos vivos: %s' % left, file=sys.stderr)
    print('\nPaso %d%s: %s' % (step, (' (' + ' '.join(params) + ')') if params else '', result))
    print('\n'.join(report.lines))
    if result == 'FAIL':
        print('  Registros: %s (sim.log y drive.log)' % logs)
        if os.path.exists(os.path.join(logs, 'drive.log')):
            tail = open(os.path.join(logs, 'drive.log')).read().strip().splitlines()[-6:]
            print('  Ultimas lineas de drive.log:\n    ' + '\n    '.join(tail))
    elif not args.keep_logs and not args.log_dir:
        subprocess.run(['rm', '-rf', logs])
    return 0 if result == 'PASS' else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0], formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('step', type=int, nargs='?', choices=range(1, 7))
    parser.add_argument('--param', action='append', default=[], help='parametro del nodo drive, p. ej. calibrate:=false')
    parser.add_argument('--seconds', type=float, help='duracion de la prueba en segundos de simulacion')
    parser.add_argument('--timeout', type=float, default=240.0, help='limite de reloj de pared (s)')
    parser.add_argument('--world', choices=('field', 'empty', 'lane'), help='campo con paredes o vacio (por defecto: vacio en 1 y 2, isla larga en 4)')
    parser.add_argument('--log-dir', help='carpeta para sim.log y drive.log (por defecto una temporal)')
    parser.add_argument('--keep-logs', action='store_true')
    parser.add_argument('--compare', action='store_true', help='paso 2: ademas ejecuta calibrate:=false para ver la deriva')
    parser.add_argument('--wait', type=float, default=0.0, help='esperar hasta N s a que otro gz sim termine')
    parser.add_argument('--empty-world', metavar='FILE', help='escribe el mundo vacio en FILE y termina')
    parser.add_argument('--lane-world', metavar='FILE', help='escribe el mundo de la isla larga en FILE y termina')
    args = parser.parse_args()
    if args.empty_world:
        print(empty_world(args.empty_world))
        return 0
    if args.lane_world:
        print(lane_world(args.lane_world))
        return 0
    if args.step is None:
        parser.error('indica el paso (1 a 6)')
    deadline = time.time() + args.wait
    while gz_sim_pids():
        if time.time() >= deadline:
            print('verify_step: ya hay un gz sim en marcha (pid %s). Cierralo o usa --wait.' % gz_sim_pids(), file=sys.stderr)
            return 2
        time.sleep(5)
    for param in args.param:
        if ':=' not in param:
            parser.error('--param debe ser nombre:=valor')
    code = run(args.step, args.param, args)
    if args.compare and args.step == 2:
        run(2, args.param + ['calibrate:=false'], args)           # shows the drift; not a pass condition
    return code


if __name__ == '__main__':
    sys.exit(main())
