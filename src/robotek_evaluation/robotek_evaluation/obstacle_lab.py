"""Obstacle lab: one Obstacle round (three laps and, with the lot, the parking) in a few seconds, without Gazebo.

The same pieces as the Gazebo rounds, so its results can be checked against them:

* the field of the rules (``obstacle_field``): walls, signs and parking-lot limits as boxes, ray casting for the
  LiDAR and a collision test of the car outline (any contact ends the round);
* the actuators of the real car (``open_lab.Actuators``, the values of ``real_car_v1.yaml``);
* the LD19 scan of ``real_car:=true`` (10 Hz, 306 rays over +-110 deg, 8 mm noise, 4.3 % dropouts, each time stamp
  delivered three times), a 50 Hz gyroscope with bias and noise, and a 6 fps camera that reports the signs in its
  field of view (66.7 deg, turned 4.4 deg left, as the real camera; 2 m at most) as the association of
  ``robotek_course.obstacle_vision`` would: ``dict(x, y, color)`` in the car frame, 1 cm noise;
* the Obstacle judge (``obstacle_judge``) on the lab trajectory: points, faults and validity mean the same thing.

What it does NOT model: wheel slip, the Ackermann plugin, camera images (so no colour mistakes), occlusion, contact
forces or signs that move. It is a one-way filter: failing here is bad news, passing proves nothing until Gazebo
agrees.

    python3 -m robotek_evaluation.obstacle_lab --strategy my_obstacle.strategy:Strategy --seed 2 --direction cw
    python3 -m robotek_evaluation.obstacle_lab --strategy my_obstacle.strategy:Strategy --parking --seed 4
    python3 -m robotek_evaluation.obstacle_lab --strategy my_obstacle.strategy:Strategy \\
        --layouts 0-4 --seeds 0-4 --early --workers 8          # 10 fields x 5 noise seeds, in parallel

The strategy is any ``robotek_course.obstacle_base.ObstacleStrategyBase`` subclass: ``on_scan(scan)``,
``on_imu(now, gyro_z)``, ``on_signs(observations, stamp)``, ``control(now, dt) -> (speed, steer)`` and ``label``,
``done``. If its constructor takes ``parking``, the lab passes whether the field has the lot.
"""
import argparse
import inspect
import json
import math
import os
from pathlib import Path
import tempfile
import time
import xml.etree.ElementTree as ET
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from robotek_common import vehicle
from .obstacle_field import build_world
from .obstacle_judge import judge_obstacle
from .open_lab import Actuators, _inside, _perimeter, actuator_parameters, apply_sets, load_strategy, outline

PHYSICS_HZ = 100.0
CONTROL_EVERY = 2            # 50 Hz, as the node
SCAN_EVERY = 10              # 10 Hz LiDAR
IMU_EVERY = 2                # 50 Hz gyroscope
CAMERA_PERIOD = 1.0 / 6.0    # 6 fps camera
TELEMETRY_EVERY = 10         # 10 Hz rows for the judge
SCAN_RAYS = 306
SCAN_HALF_ANGLE = math.radians(110.0)
SCAN_MIN_RANGE = 0.12
SCAN_NOISE = 0.008
SCAN_DROPOUT = 0.043
SCAN_COPIES = 3
SCAN_MAX_RANGE = 4.0
GYRO_NOISE = 0.0019
GYRO_BIAS = 0.006
CAMERA_RANGE = 2.0           # m; colours are only reported this far
CAMERA_NOISE = 0.01          # m on each coordinate of an observation
STOP_HOLD = 31.0             # the judge needs the stop observed for 30 s
FAULT_GRACE = 1.5            # s simulated after a clear wrong-side pass before stopping (--early)
CONTROL_PERIOD = CONTROL_EVERY / PHYSICS_HZ


# ------------------------------------------------------------------- world
class World:
    """Walls, signs and parking-lot limits of one round folder (``world.sdf`` + ``manifest.json``)."""

    def __init__(self, directory):
        self.directory = Path(directory)
        self.manifest = json.loads((self.directory / 'manifest.json').read_text())
        self.pillars = self.manifest['pillars']
        self.lot = self.manifest.get('parking_lot')
        root = ET.parse(self.directory / 'world.sdf').getroot()
        boxes, kinds = [], []
        for node in root.findall(".//model[@name='game_field']//collision"):
            name = node.get('name', '')
            if name.startswith('wall_'):
                pose = [float(v) for v in node.findtext('pose', '0 0 0 0 0 0').split()]
                size = [float(v) for v in node.findtext('geometry/box/size').split()]
                boxes.append((pose[0] - size[0] / 2, pose[0] + size[0] / 2, pose[1] - size[1] / 2, pose[1] + size[1] / 2))
                kinds.append('inner_wall' if name.startswith('wall_int') else 'outer_wall')
        for link in root.findall(".//model[@name='parking_lot']/link"):
            x, y = [float(v) for v in link.findtext('pose').split()[:2]]
            sx, sy = [float(v) for v in link.find('collision/geometry/box/size').text.split()[:2]]
            boxes.append((x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2))
            kinds.append('parking_limit')
        self.wall_boxes = np.array(boxes, dtype=float)
        self.kinds = kinds
        half = 0.025
        self.sign_boxes = np.array([(p['x'] - half, p['x'] + half, p['y'] - half, p['y'] + half)
                                    for p in self.pillars], dtype=float).reshape(-1, 4)

    def raycast(self, ox, oy, angles):
        best = np.full(angles.shape, np.inf)
        for boxes in (self.wall_boxes, self.sign_boxes):
            for xmin, xmax, ymin, ymax in boxes:
                best = np.minimum(best, _ray_box(ox, oy, angles, xmin, xmax, ymin, ymax))
        return best

    def clearances(self, polygon):
        """Smallest distance from the car outline to each kind of obstacle (<= 0 touches)."""
        out = {'outer_wall': math.inf, 'inner_wall': math.inf, 'pillar': math.inf}
        if self.lot:
            out['parking_limit'] = math.inf
        samples = _perimeter(polygon, 0.02)
        for box, kind in zip(self.wall_boxes, self.kinds):
            out[kind] = min(out[kind], _box_distance(samples, polygon, *box))
        for box in self.sign_boxes:
            out['pillar'] = min(out['pillar'], _box_distance(samples, polygon, *box))
        return out


def _ray_box(ox, oy, angles, xmin, xmax, ymin, ymax):
    dx, dy = np.cos(angles), np.sin(angles)
    with np.errstate(divide='ignore', invalid='ignore'):
        tx = np.stack([(xmin - ox) / dx, (xmax - ox) / dx])
        ty = np.stack([(ymin - oy) / dy, (ymax - oy) / dy])
        near = np.maximum(tx.min(axis=0), ty.min(axis=0))
        far = np.minimum(tx.max(axis=0), ty.max(axis=0))
        hit = (far >= near) & (near > 1e-6)
    return np.where(hit, near, np.inf)


def _box_distance(samples, polygon, xmin, xmax, ymin, ymax):
    dx = np.maximum(np.maximum(xmin - samples[:, 0], samples[:, 0] - xmax), 0.0)
    dy = np.maximum(np.maximum(ymin - samples[:, 1], samples[:, 1] - ymax), 0.0)
    distance = float(np.hypot(dx, dy).min())
    corners = np.array([(xmin, ymin), (xmin, ymax), (xmax, ymin), (xmax, ymax)])
    if _inside(polygon, corners).any():
        distance = 0.0
    return distance


class SideWatch:
    """Early warning of a clear wrong-side pass (the car's centre crosses a sign's radius line on the forbidden
    side), for ``--early``; the judge still decides."""

    TANGENTS = {'bottom': (1, 0), 'right': (0, 1), 'top': (-1, 0), 'left': (0, -1)}

    def __init__(self, pillars, direction):
        sign = 1 if direction == 'ccw' else -1
        self.signs = [(p, self.TANGENTS[p['side']][0] * sign, self.TANGENTS[p['side']][1] * sign) for p in pillars]
        self.before = {p['name']: False for p in pillars}

    def wrong(self, x, y):
        for p, tx, ty in self.signs:
            horizontal = p['side'] in ('bottom', 'top')
            near = (abs(x - p['x']) < .6 and y * math.copysign(1, p['y']) >= .45) if horizontal else \
                (abs(y - p['y']) < .6 and x * math.copysign(1, p['x']) >= .45)
            if not near:
                continue
            along = tx * (x - p['x']) + ty * (y - p['y'])
            if along < 0:
                self.before[p['name']] = True
            elif self.before[p['name']]:
                self.before[p['name']] = False
                lateral = -ty * (x - p['x']) + tx * (y - p['y'])
                if (lateral > 0) == (p['color'] == 'red'):
                    return True
        return False


# ----------------------------------------------------------------- sensors
def make_scan(ranges, now, angle_min, increment):
    from types import SimpleNamespace
    sec = int(now)
    return SimpleNamespace(
        ranges=ranges, angle_min=angle_min, angle_increment=increment,
        range_min=SCAN_MIN_RANGE, range_max=SCAN_MAX_RANGE,
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=int(round((now - sec) * 1e9)))))


def camera_observations(world, x, y, yaw, rng, max_range=CAMERA_RANGE):
    """Signs in the real camera's field of view, as the colour-LiDAR association reports them."""
    observations = []
    c, s = math.cos(yaw), math.sin(yaw)
    for pillar in world.pillars:
        dx, dy = pillar['x'] - x, pillar['y'] - y
        bx, by = c * dx + s * dy, -s * dx + c * dy
        distance = math.hypot(bx - vehicle.CAMERA_OFFSET_X, by)
        if not SCAN_MIN_RANGE < distance < max_range:
            continue
        bearing = math.atan2(by, bx - vehicle.CAMERA_OFFSET_X)
        if abs(bearing - vehicle.REAL_CAMERA_YAW) > vehicle.REAL_CAMERA_HFOV / 2:
            continue
        observations.append(dict(x=bx + rng.normal(0.0, CAMERA_NOISE), y=by + rng.normal(0.0, CAMERA_NOISE),
                                 color=pillar['color']))
    return observations


# ------------------------------------------------------------------- round
def make_strategy(spec, parking, overrides=None):
    """Zero-argument factory for ``module:Class``; ``parking`` goes to the constructor if it takes it."""
    factory = load_strategy(spec, overrides)
    import importlib
    module, _, name = spec.partition(':')
    cls = getattr(importlib.import_module(module), name or 'Strategy')
    if 'parking' in inspect.signature(cls).parameters:
        return lambda: cls(**{**(overrides or {}), 'parking': parking})
    return factory


def run_round(directory, strategy, seed=0, tmax=180.0, stop_on_fault=False, trace=False):
    """One round on the field of ``directory`` (``build_world``); ``strategy`` is a zero-argument factory."""
    world = World(directory)
    direction = world.manifest['direction']
    spawn = world.manifest['spawn']
    strategy = strategy()
    actuators = Actuators(actuator_parameters())
    rng = np.random.default_rng(seed)
    yaw = spawn['yaw']
    back = -vehicle.REAR_AXLE_X
    xr, yr = spawn['x'] - back * math.cos(yaw), spawn['y'] - back * math.sin(yaw)
    x, y = spawn['x'], spawn['y']
    angle_min = -SCAN_HALF_ANGLE
    increment = -2.0 * angle_min / (SCAN_RAYS - 1)
    beams = angle_min + increment * np.arange(SCAN_RAYS)
    bias = GYRO_BIAS + rng.normal(0.0, 0.0005)
    dt = 1.0 / PHYSICS_HZ
    rows, events = [], [{'event': 'start', 'sim_time': 0.0, 'wall_time': 0.0}]
    camera_time = scan_time = None
    contact = contact_kind = None
    clearance = {k: math.inf for k in world.clearances(outline(x, y, yaw))}
    sides = SideWatch(world.pillars, direction) if stop_on_fault else None
    fault_at = stopped_at = None
    steer_max = yaw_rate = 0.0
    for tick in range(int(tmax * PHYSICS_HZ)):
        now = tick * dt
        if tick % SCAN_EVERY == 0:
            ranges = world.raycast(x + vehicle.LIDAR_OFFSET_X * math.cos(yaw),
                                   y + vehicle.LIDAR_OFFSET_X * math.sin(yaw), beams + yaw)
            ranges = ranges + rng.normal(0.0, SCAN_NOISE, SCAN_RAYS)
            ranges[(ranges < SCAN_MIN_RANGE) | ~np.isfinite(ranges)] = math.inf
            for _ in range(SCAN_COPIES):
                copy = ranges.copy()
                copy[rng.random(SCAN_RAYS) < SCAN_DROPOUT] = math.inf
                strategy.on_scan(make_scan(list(copy), now, angle_min, increment))
            scan_time = now
        if tick % IMU_EVERY == 0:
            strategy.on_imu(now, yaw_rate + bias + rng.normal(0.0, GYRO_NOISE))
        if tick % CONTROL_EVERY == 0:
            # The node only associates colours with a scan no older than 0.15 s.
            if (camera_time is None or now - camera_time >= CAMERA_PERIOD - 1e-9) and scan_time is not None \
                    and now - scan_time <= 0.15:
                camera_time = now
                strategy.on_signs(camera_observations(world, x, y, yaw, rng), now)
            speed, steer = strategy.control(now, CONTROL_PERIOD)
            actuators.command(now, speed, steer)
        v_out, yaw_rate = actuators.step(now, dt)
        yaw += yaw_rate * dt
        xr += v_out * math.cos(yaw) * dt
        yr += v_out * math.sin(yaw) * dt
        x, y = xr + back * math.cos(yaw), yr + back * math.sin(yaw)
        steer_max = max(steer_max, abs(actuators.steer))
        if tick % CONTROL_EVERY == 0:
            current = world.clearances(outline(x, y, yaw))
            clearance = {k: min(clearance[k], current[k]) for k in clearance}
            if contact is None and min(current.values()) <= 0.0:
                contact, contact_kind = now, min(current, key=current.get)
                contact_pillar = _nearest_pillar(world, x, y) if contact_kind == 'pillar' else None
        if tick % TELEMETRY_EVERY == 0:
            rows.append(_row(now, x, y, yaw, actuators.steer))
            if sides is not None and fault_at is None and sides.wrong(x, y):
                fault_at = now
        if contact is not None or (fault_at is not None and now - fault_at > FAULT_GRACE):
            break
        if strategy.done and stopped_at is None:
            stopped_at = now
            events.append({'event': 'done', 'sim_time': now, 'wall_time': now})
        if stopped_at is not None and abs(v_out) < 1e-3 and now - stopped_at > 1.0:
            break
    if stopped_at is not None and contact is None and rows:
        last = rows[-1]
        count = int(STOP_HOLD * PHYSICS_HZ / TELEMETRY_EVERY)
        rows.extend(_row(last['sim_time'] + (i + 1) * TELEMETRY_EVERY / PHYSICS_HZ, x, y, yaw, actuators.steer)
                    for i in range(count))
    info = None if contact is None else dict(sim_time=contact, kind=contact_kind,
                                             pillar=contact_pillar if contact_kind == 'pillar' else None)
    result = _judge(world, rows, events, info, steer_max)
    result.update(run=str(directory), direction=direction, contact=info, end=strategy.label,
                  measured_clearance=min(clearance.values()))
    if trace:
        result['rows'] = rows
    return result


def _nearest_pillar(world, x, y):
    return min(world.pillars, key=lambda p: math.hypot(p['x'] - x, p['y'] - y))['name']


def _row(now, x, y, yaw, steer):
    return {'sim_time': now, 'wall_time': now, 'x_gt': x, 'y_gt': y, 'yaw_gt': yaw,
            'steer_actual': steer, 'steer_right_actual': steer}


def _judge(world, rows, events, contact, steer_max):
    """The Obstacle judge on the lab trajectory, written as a round folder."""
    import shutil
    with tempfile.TemporaryDirectory() as tmp:
        directory = Path(tmp)
        contacts = []
        if contact:
            other = {'inner_wall': 'game_field::field::wall_int', 'outer_wall': 'game_field::field::wall_ext',
                     'parking_limit': 'parking_lot::parking_limit'}.get(contact['kind'], 'traffic_signs::%s' % contact['pillar'])
            contacts.append(dict(sim_time=contact['sim_time'], wall_time=contact['sim_time'], source='lab',
                                 pillar=contact['pillar'], other=other, kind=contact['kind'],
                                 robot='robotek::base_footprint', max_depth=0.001, points=1, episode=0,
                                 end_sim_time=contact['sim_time'], messages=1))
        (directory / 'contacts.jsonl').write_text(''.join(json.dumps(c) + '\n' for c in contacts))
        shutil.copyfile(world.directory / 'world.sdf', directory / 'world.sdf')
        lines = []
        for row in rows[::10]:
            poses = [{'name': 'traffic_signs', 'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}]
            poses += [{'name': 'traffic_signs/' + p['name'], 'x': p['x'], 'y': p['y'], 'z': 0.05, 'yaw': 0.0}
                      for p in world.pillars]
            lines.append(json.dumps({'sim_time': row['sim_time'], 'poses': poses}) + '\n')
        (directory / 'pillars-gt.jsonl').write_text(''.join(lines))
        report = judge_obstacle(directory, world.manifest, rows, events)
    faults = report['rule_judge']['faults']
    return {'points': report['points_estimated'], 'max_points': report['rule_judge']['max_points'],
            'time': report['rule_judge']['time_from_start_sim_seconds'],
            'valid': report['validation']['passed'], 'failed': report['validation']['failed'],
            'margin': report['validation']['margin_class'], 'faults': faults,
            'breakdown': report['points_breakdown_estimated'], 'laps': report['laps_completed'],
            'parking': (report.get('parking') or {}).get('verdict'), 'steer_max_deg': math.degrees(steer_max)}


# ------------------------------------------------------------------- batch
def make_field(base, layout, direction, parking):
    directory = Path(base) / ('obstacle-%d-%s%s' % (layout, direction, '-park' if parking else ''))
    if not (directory / 'manifest.json').exists():
        build_world(directory, layout, parking, direction)
    return directory


def _job(args):
    spec, directory, seed, parking, sets, early, tmax = args
    if sets:
        apply_sets(sets)
    started = time.process_time()
    result = run_round(directory, make_strategy(spec, parking), seed=seed, tmax=tmax, stop_on_fault=early)
    result.update(seed=seed, cpu=round(time.process_time() - started, 1))
    return result


def _span(text):
    out = []
    for part in text.split(','):
        low, _, high = part.partition('-')
        out.extend(range(int(low), int(high or low) + 1))
    return out


def format_result(r):
    reason = ''
    if not r['valid']:
        reason = ('%s:%s' % (r['faults'][0]['fault'], r['faults'][0].get('pillar')) if r['faults'] else
                  ('contact:%s' % (r['contact']['pillar'] or r['contact']['kind']) if r['contact'] else
                   ','.join(r['failed'])))
    return '%-22s s%-2d pts=%2s/%-2s t=%-6s %-7s %s%s' % (
        Path(r['run']).name, r['seed'], r['points'], r['max_points'], '%.1f' % r['time'] if r['time'] else '-',
        'VALID' if r['valid'] else 'invalid', reason, ' park=%s' % r['parking'] if r['parking'] else '')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--strategy', required=True, help='module:Class, e.g. my_obstacle.strategy:Strategy')
    parser.add_argument('--seed', type=int, help='layout of one round (default: --layouts)')
    parser.add_argument('--layouts', default='0-4', help='layouts of a batch, e.g. 0-4 or 0,3,7')
    parser.add_argument('--direction', choices=['ccw', 'cw'], help='one direction (default: both)')
    parser.add_argument('--seeds', default='0', help='noise seeds, e.g. 0-4')
    parser.add_argument('--parking', action='store_true', help='fields with the parking lot (the car parks)')
    parser.add_argument('--early', action='store_true', help='stop a round 1.5 s after a clear wrong-side pass')
    parser.add_argument('--tmax', type=float, default=180.0)
    parser.add_argument('--set', action='append', default=[], help='module.NAME=value (see open_lab.apply_sets)')
    parser.add_argument('--workers', type=int, default=max(1, (os.cpu_count() or 2) - 1))
    parser.add_argument('--fields-dir', default=str(Path(tempfile.gettempdir()) / 'robotek-obstacle-fields'))
    parser.add_argument('--out', type=Path, help='write the results as JSON')
    args = parser.parse_args(argv)
    layouts = [args.seed] if args.seed is not None else _span(args.layouts)
    directions = [args.direction] if args.direction else ['ccw', 'cw']
    jobs = [(args.strategy, str(make_field(args.fields_dir, layout, direction, args.parking)), seed, args.parking,
             args.set, args.early, args.tmax)
            for layout in layouts for direction in directions for seed in _span(args.seeds)]
    started = time.time()
    if len(jobs) == 1 or args.workers <= 1:
        results = [_job(job) for job in jobs]
    else:
        with ProcessPoolExecutor(max_workers=args.workers) as pool:
            results = list(pool.map(_job, jobs))
    for r in results:
        print(format_result(r), flush=True)
    valid = [r for r in results if r['valid']]
    times = [r['time'] for r in valid if r['time']]
    print('valid %d/%d  mean time of the valid %.1f s  wall %.0f s' % (
        len(valid), len(results), sum(times) / len(times) if times else 0.0, time.time() - started))
    if args.out:
        args.out.write_text(json.dumps(results, indent=1, default=float))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
