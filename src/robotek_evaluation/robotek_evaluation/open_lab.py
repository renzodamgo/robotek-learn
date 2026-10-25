"""Open lab: the whole Open round in a few seconds, without Gazebo.

A closed-loop simulator for comparing Open strategies fast, built from the same pieces
the Gazebo runs use so that its results can be checked against them:

* the real field geometry (the wall boxes of the SDF), with ray casting for the LiDAR and
  a collision / clearance test of the car outline against the walls;
* the actuators of the real car: the same slew, delay, lag and speed table of
  ``sim_actuator_model`` with the values of ``real_car_v1.yaml`` (and the wheelbase default
  that model really uses in Gazebo);
* the LD19 scan of ``real_car:=true`` (10 Hz, 306 rays over +-110 deg, 8 mm noise, 4.3 %
  dropouts), a 50 Hz gyroscope with bias and noise, a 6 fps camera that knows the direction
  after ``camera_delay`` seconds (the strategy brings its own ``LidarOdometry`` for the distance driven);
* the rule judge of the Gazebo rounds (``judge_open`` and ``standard_validation``) applied to
  the lab trajectory, so points, stop and validity mean the same thing.

What it does NOT model: wheel slip, the Ackermann plugin dynamics, camera images, contact
forces (a contact ends the round) or the motor under load. Results must be compared with
Gazebo before they are used to decide anything.

    python3 -m robotek_evaluation.open_lab --strategy my_open.strategy:Strategy
    python3 -m robotek_evaluation.open_lab --strategy my_open.strategy:Strategy --field square100 --direction ccw
"""
import argparse
import json
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from robotek_common import vehicle
from .rule_judge import judge_open, standard_validation
from .sim_math import slew, table_speed



def fields_dir():
    """The folder with the Open field worlds: robotek_gazebo's installed share, or the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('robotek_gazebo')) / 'worlds'
        if path.is_dir():
            return path
    except Exception:  # not a ROS environment: fall back to the repository sources
        pass
    return Path(__file__).resolve().parents[2] / 'robotek_gazebo' / 'worlds'


FIELDS = fields_dir()
STD12 = tuple((field, direction) for field in (
    'square100', 'square180', 'rect100x180', 'rect180x100', 'shift140', 'east140x100')
    for direction in ('ccw', 'cw'))
START = (0.0, -1.15)

PHYSICS_HZ = 100.0          # the actuator model runs at 100 Hz in Gazebo too
CONTROL_EVERY = 2           # 50 Hz strategy
SCAN_EVERY = 10             # 10 Hz LiDAR
IMU_EVERY = 2               # 50 Hz gyroscope
CAMERA_PERIOD = 1.0 / 6.0
TELEMETRY_EVERY = 10        # 10 Hz rows for the judge, as the Gazebo telemetry
SCAN_RAYS = 306
SCAN_HALF_ANGLE = math.radians(110.0)
SCAN_MIN_RANGE = 0.12
SCAN_NOISE = 0.008
SCAN_DROPOUT = 0.043
# The Gazebo runs deliver every LiDAR stamp three times (recorded scans: 30 messages/s for a 10 Hz
# sensor), each copy with its own dropouts.
SCAN_COPIES = 3
GYRO_NOISE = 0.0019
GYRO_BIAS = 0.006
# Optional first-order lag between the actuator model's yaw rate and the body's. Off by default: against
# the Gazebo ground truth the yaw rate follows the measured wheel angle with no lag (rms 0.03 rad/s);
# the lag that exists is in the wheel itself. A fit of 0.05 s lowers the open-loop error but makes
# the baseline rounds 1.5 s slower than Gazebo, so it is left to experiments.
PLANT_TAU = 0.0
CAMERA_DELAY = 0.3          # s after Start until the camera has confirmed the direction (Gazebo: before 0.44 s)
STOP_HOLD = 31.0            # the judge needs the stop observed for 30 s


# ------------------------------------------------------------------- world
class World:
    """Axis-aligned wall boxes read from an Open field SDF."""

    def __init__(self, path):
        link = ET.parse(path).find(".//model[@name='game_field']/link[@name='field']")
        boxes, kinds = [], []
        for node in link.findall('collision'):
            name = node.get('name', '')
            if not name.startswith('wall_'):
                continue
            pose = [float(v) for v in node.findtext('pose').split()]
            size = [float(v) for v in node.findtext('geometry/box/size').split()]
            boxes.append((pose[0] - size[0] / 2, pose[0] + size[0] / 2,
                          pose[1] - size[1] / 2, pose[1] + size[1] / 2))
            kinds.append('inner_wall' if name.startswith('wall_int') else 'outer_wall')
        self.boxes = np.array(boxes)
        self.kinds = kinds
        self.path = Path(path)
        from .open_field_geometry import field_geometry
        self.geometry = field_geometry(path)

    def raycast(self, ox, oy, angles):
        """Distance from (ox, oy) along each angle to the nearest wall box (inf if none)."""
        dx, dy = np.cos(angles), np.sin(angles)
        best = np.full(angles.shape, np.inf)
        with np.errstate(divide='ignore', invalid='ignore'):
            for xmin, xmax, ymin, ymax in self.boxes:
                tx = np.stack([(xmin - ox) / dx, (xmax - ox) / dx])
                ty = np.stack([(ymin - oy) / dy, (ymax - oy) / dy])
                near = np.maximum(tx.min(axis=0), ty.min(axis=0))
                far = np.minimum(tx.max(axis=0), ty.max(axis=0))
                # A ray that starts inside a box (the car touching a wall) does not see it.
                hit = (far >= near) & (near > 1e-6)
                best = np.where(hit & (near < best), near, best)
        return best

    def clearances(self, polygon):
        """(outer, inner) smallest distance from the car outline to each kind of wall; <= 0 touches."""
        out = {'outer_wall': math.inf, 'inner_wall': math.inf}
        samples = _perimeter(polygon, 0.02)
        for (xmin, xmax, ymin, ymax), kind in zip(self.boxes, self.kinds):
            dx = np.maximum(np.maximum(xmin - samples[:, 0], samples[:, 0] - xmax), 0.0)
            dy = np.maximum(np.maximum(ymin - samples[:, 1], samples[:, 1] - ymax), 0.0)
            distance = float(np.hypot(dx, dy).min())
            corners = np.array([(xmin, ymin), (xmin, ymax), (xmax, ymin), (xmax, ymax)])
            if _inside(polygon, corners).any():
                distance = 0.0
            out[kind] = min(out[kind], distance)
        return out


def _perimeter(polygon, step):
    points = []
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        count = max(2, int(math.dist(a, b) / step))
        for i in range(count):
            f = i / count
            points.append((a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f))
    return np.array(points)


def _inside(polygon, points):
    """Points inside a convex counter-clockwise polygon."""
    inside = np.ones(len(points), dtype=bool)
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        cross = (b[0] - a[0]) * (points[:, 1] - a[1]) - (b[1] - a[1]) * (points[:, 0] - a[0])
        inside &= cross >= 0.0
    return inside


def outline(x, y, yaw):
    """Car outline (counter-clockwise) from the measured overall dimensions."""
    c, s = math.cos(yaw), math.sin(yaw)
    corners = [(vehicle.OUTLINE_FRONT, -vehicle.OUTLINE_HALF_WIDTH),
               (vehicle.OUTLINE_FRONT, vehicle.OUTLINE_HALF_WIDTH),
               (-vehicle.OUTLINE_REAR, vehicle.OUTLINE_HALF_WIDTH),
               (-vehicle.OUTLINE_REAR, -vehicle.OUTLINE_HALF_WIDTH)]
    return [(x + px * c - py * s, y + px * s + py * c) for px, py in corners]


# --------------------------------------------------------------- actuators
def _config_path():
    """real_car_v1.yaml: the installed share folder of this package, or the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('robotek_evaluation')) / 'config' / 'real_car_v1.yaml'
        if path.is_file():
            return path
    except Exception:
        pass
    return Path(__file__).resolve().parents[1] / 'config' / 'real_car_v1.yaml'


def actuator_parameters(path=None):
    """The sim_actuator_model parameters of the real car, with that node's own defaults."""
    path = path or _config_path()
    values = yaml.safe_load(Path(path).read_text())['sim_actuator_model']['ros__parameters']
    # Measured in Gazebo with scripted steering steps: the wheel starts moving 0.10 s
    # after the command (the model's 0.06 s plus the plugin's) and follows with a 0.06 s lag.
    values = {**values, 'steer_delay': 0.10, 'steer_tau': 0.06}
    defaults = dict(wheelbase=0.13, steer_limit=0.4363323129985824, steer_delay=0.03,
                    steer_rate=4.4, steer_tau=0.03, accel_max=2.0, decel_max=2.0,
                    drive_delay=0.0, drive_tau=0.0, drive_command_limit=0.0, drive_deadband=0.0,
                    speed_table_cmd=[0.0], speed_table_real=[0.0])
    return {**defaults, **values}


class Actuators:
    """Python mirror of SimActuatorModel: servo with dead time, rate and lag; motor with slew,
    dead time, speed table and lag. Input: speed and steering commands. Output: speed and yaw rate."""

    def __init__(self, params=None):
        self.p = params or actuator_parameters()
        cmd = [float(x) for x in self.p['speed_table_cmd'] if float(x) > 0.0]
        real = [float(x) for x in self.p['speed_table_real']][-len(cmd):] if cmd else []
        self.table = (cmd, real)
        self.v_cmd = self.steer_cmd = 0.0
        self.v = self.steer = self.v_out = 0.0
        self.target = 0.0
        self.steer_queue, self.drive_queue = [], []
        self.delayed_v = 0.0

    def command(self, now, speed, steer):
        self.v_cmd = speed
        if abs(speed) > 1e-3:
            # The node turns (v, steer) into a Twist, the model back into a steering angle with
            # its own wheelbase.
            wz = speed * math.tan(steer) / vehicle.WHEELBASE
            self.steer_cmd = math.atan(wz * self.p['wheelbase'] / speed)
        self.steer_queue.append((now, self.steer_cmd))

    def step(self, now, dt):
        p = self.p
        target = None
        while self.steer_queue and now - self.steer_queue[0][0] >= p['steer_delay']:
            target = self.steer_queue.pop(0)[1]
        if target is not None:
            self.target = max(-p['steer_limit'], min(p['steer_limit'], target))
        change = self.target - self.steer
        if p['steer_tau'] > 0.0:
            change *= min(1.0, dt / p['steer_tau'])
        limit = p['steer_rate'] * dt
        self.steer += max(-limit, min(limit, change))
        self.v = slew(self.v, self.v_cmd, p['accel_max'], p['decel_max'], dt)
        v = self.v
        if p['drive_delay'] > 0.0:
            self.drive_queue.append((now, v))
            v = None
            while self.drive_queue and now - self.drive_queue[0][0] >= p['drive_delay']:
                v = self.drive_queue.pop(0)[1]
            self.delayed_v = self.delayed_v if v is None else v
            v = self.delayed_v
        if p['drive_command_limit'] > 0.0:
            v = max(-p['drive_command_limit'], min(p['drive_command_limit'], v))
        if abs(v) < p['drive_deadband']:
            v = 0.0
        v = table_speed(v, *self.table)
        if p['drive_tau'] > 0.0:
            self.v_out += (v - self.v_out) * min(1.0, dt / p['drive_tau'])
        else:
            self.v_out = v
        return self.v_out, self.v_out * math.tan(self.steer) / p['wheelbase']


# ---------------------------------------------------------------- strategy
def load_strategy(spec, overrides=None):
    """Zero-argument factory for ``module:Class`` (for example ``my_open.strategy:Strategy``).

    The class must offer what ``run_round`` calls: ``on_scan(scan) -> bool`` (True on the first scan, the Start),
    ``on_imu(now, gyro_z)``, ``on_camera(direction)`` (+1 counter-clockwise, -1 clockwise, 0 unknown),
    ``control(dt, camera_age) -> (speed, steer)`` and the properties ``label``, ``turns``, ``done`` and
    ``stop_label``. ``overrides`` are passed to the constructor as keyword arguments.
    """
    import importlib
    module, _, name = spec.partition(':')
    cls = getattr(importlib.import_module(module), name or 'Strategy')
    return lambda: cls(**(overrides or {}))


# ------------------------------------------------------------------- round
def make_scan(ranges, now, angle_min, increment):
    from types import SimpleNamespace
    sec = int(now)
    return SimpleNamespace(
        ranges=ranges, angle_min=angle_min, angle_increment=increment, range_min=SCAN_MIN_RANGE,
        range_max=12.0, header=SimpleNamespace(stamp=SimpleNamespace(
            sec=sec, nanosec=int(round((now - sec) * 1e9)))))


def run_round(world, direction, strategy, seed=0, tmax=180.0, spawn=START, plant_tau=PLANT_TAU,
              actuator=None, scan_copies=SCAN_COPIES, trace=False, latency=0.0):
    """One round: ``strategy`` is a zero-argument factory. Returns the lab result dict."""
    rng = np.random.default_rng(seed)
    world = world if isinstance(world, World) else World(world)
    strategy = strategy()
    actuators = Actuators({**actuator_parameters(), **(actuator or {})})
    sign = 1.0 if direction == 'ccw' else -1.0
    yaw = 0.0 if direction == 'ccw' else math.pi
    # The car turns about its rear axle (bicycle model): the pose of base_link, at mid-wheelbase, slides
    # sideways by atan(REAR_AXLE_X*yaw_rate/v) (0.14 rad at full lock; measured in Gazebo).
    back = -vehicle.REAR_AXLE_X
    xr, yr = spawn[0] - back * math.cos(yaw), spawn[1] - back * math.sin(yaw)
    x, y = spawn
    angle_min = -SCAN_HALF_ANGLE
    increment = -2.0 * angle_min / (SCAN_RAYS - 1)
    beams = angle_min + increment * np.arange(SCAN_RAYS)
    bias = GYRO_BIAS + rng.normal(0.0, 0.0005)
    dt = 1.0 / PHYSICS_HZ
    rows, events = [], []
    t_start = None
    camera_time, camera_stamp = None, None
    camera_direction = 0
    contact = None
    clearance = {'outer_wall': math.inf, 'inner_wall': math.inf}
    steer_max = 0.0
    yaw_rate = 0.0
    stopped_at = None
    commanded_rate = 0.0
    pending = []
    for tick in range(int(tmax / dt)):
        now = tick * dt
        if tick % SCAN_EVERY == 0:
            ranges = world.raycast(x + vehicle.LIDAR_OFFSET_X * math.cos(yaw),
                                   y + vehicle.LIDAR_OFFSET_X * math.sin(yaw), beams + yaw)
            ranges = ranges + rng.normal(0.0, SCAN_NOISE, SCAN_RAYS)
            ranges[(ranges < SCAN_MIN_RANGE) | ~np.isfinite(ranges)] = math.inf
            for _ in range(scan_copies):
                copy = ranges.copy()
                copy[rng.random(SCAN_RAYS) < SCAN_DROPOUT] = math.inf
                if strategy.on_scan(make_scan(list(copy), now, angle_min, increment)):
                    t_start = now
                    events.append({'event': 'start', 'sim_time': now, 'wall_time': now})
        if tick % IMU_EVERY == 0:
            strategy.on_imu(now, yaw_rate + bias + rng.normal(0.0, GYRO_NOISE))
        if t_start is not None and now >= t_start + CAMERA_DELAY and \
                (camera_time is None or now - camera_time >= CAMERA_PERIOD - 1e-9):
            camera_time = now
            camera_direction = int(sign)
            strategy.on_camera(camera_direction)
        if tick % CONTROL_EVERY == 0:
            age = math.inf if camera_time is None else now - camera_time
            speed, steer = strategy.control(dt * CONTROL_EVERY, age)
            pending.append((now + latency, speed, steer))
        while pending and pending[0][0] <= now + 1e-9:
            actuators.command(now, *pending.pop(0)[1:])
        v_out, commanded_rate = actuators.step(now, dt)
        yaw_rate += (commanded_rate - yaw_rate) * (min(1.0, dt / plant_tau) if plant_tau > 0 else 1.0)
        yaw += yaw_rate * dt
        xr += v_out * math.cos(yaw) * dt
        yr += v_out * math.sin(yaw) * dt
        x, y = xr + back * math.cos(yaw), yr + back * math.sin(yaw)
        steer_max = max(steer_max, abs(actuators.steer))
        if tick % CONTROL_EVERY == 0:
            clearance = {k: min(clearance[k], v) for k, v in world.clearances(outline(x, y, yaw)).items()}
            if contact is None and min(clearance.values()) <= 0.0:
                contact = now
        if tick % TELEMETRY_EVERY == 0:
            rows.append(_row(now, strategy.label, x, y, yaw, actuators.steer))
        if contact is not None:
            break
        if strategy.done and stopped_at is None:
            stopped_at = now
            events.append({'event': 'done', 'sim_time': now, 'wall_time': now})
        if stopped_at is not None and abs(v_out) < 1e-3 and abs(yaw_rate) < 1e-3 \
                and now - stopped_at > 1.0:
            break
    last = rows[-1] if rows else None
    if stopped_at is not None and contact is None and last is not None:
        # Stopped for good: the car stays where it is while the judge's 30 s observation passes.
        count = int(STOP_HOLD * PHYSICS_HZ / TELEMETRY_EVERY)
        rows.extend(_row(last['sim_time'] + (i + 1) * TELEMETRY_EVERY / PHYSICS_HZ, last['state'],
                         x, y, yaw, actuators.steer) for i in range(count))
    result = _result(world, direction, strategy, rows, events, clearance, contact, steer_max, t_start)
    if trace:
        result['rows'] = rows
    return result


def _row(now, label, x, y, yaw, steer):
    return {'sim_time': now, 'wall_time': now, 'state': label, 'x_gt': x, 'y_gt': y, 'yaw_gt': yaw,
            'steer_actual': steer, 'steer_right_actual': steer}


def _result(world, direction, strategy, rows, events, clearance, contact, steer_max, t_start):
    with tempfile.TemporaryDirectory() as tmp:
        manifest = {'direction': direction, 'field_geometry': world.geometry}
        judge = judge_open(Path(tmp), manifest, rows, events)
        validation = standard_validation(
            judge, {k: (None if math.isinf(v) else v) for k, v in clearance.items()},
            contacts_evaluated=True, contacts_any=contact is not None,
            stop_hold=judge.get('stop_hold_sim_seconds'), steer_max_deg=math.degrees(steer_max))
    return {'field': world.path.stem, 'direction': direction, 'points': judge['points'],
            'time': judge['time_from_start_sim_seconds'], 'valid': validation['passed'],
            'failed': validation['failed'], 'margin': validation['margin_class'],
            'min_clearance': validation['min_clearance_m'], 'contact_time': contact,
            'end': strategy.stop_label or strategy.label, 'turns': strategy.turns,
            'laps': [lap['sim_seconds'] for lap in judge['lap_splits']]}


# --------------------------------------------------------------------- CLI
def run_suite(strategy, seed=0, fields=None, directions=None, worlds=FIELDS):
    results = []
    for field, direction in STD12:
        if (fields and field not in fields) or (directions and direction not in directions):
            continue
        results.append(run_round(worlds / f'{field}.sdf', direction, strategy, seed=seed))
    return results


def format_result(r):
    time = '%.2f' % r['time'] if r['time'] else '-'
    return '%-22s pts=%-3s t=%-7s valid=%-5s %-12s %s' % (
        f"{r['field']}-{r['direction']}", r['points'], time, r['valid'], r['margin'],
        ','.join(r['failed']))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--suite', default='std12', choices=['std12'])
    parser.add_argument('--tag', default='lab')
    parser.add_argument('--strategy', required=True, help='module:Class, for example my_open.strategy:Strategy')
    parser.add_argument('--param', action='append', default=[], help='name=value, passed to the constructor')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--field', action='append')
    parser.add_argument('--direction', action='append')
    parser.add_argument('--out', type=Path, help='write the results as JSON')
    args = parser.parse_args(argv)
    overrides = {k: yaml.safe_load(v) for k, v in (p.split('=', 1) for p in args.param)}
    factory = load_strategy(args.strategy, overrides)
    results = run_suite(factory, args.seed, args.field, args.direction)
    for result in results:
        print(format_result(result), flush=True)
    valid = sum(r['valid'] for r in results)
    print(f'{valid}/{len(results)} valid, mean time '
          f'{np.mean([r["time"] for r in results if r["time"]]):.2f} s')
    if args.out:
        args.out.write_text(json.dumps({'tag': args.tag, 'seed': args.seed, 'rounds': results}, indent=1))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
