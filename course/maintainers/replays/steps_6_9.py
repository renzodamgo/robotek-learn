"""Replay trajectories for the guide's Three.js player: steps 6 to 9 (files step-6.json .. step-9.json).

Every file is one closed-loop round of the Open lab (robotek_evaluation.open_lab), with the real-car actuators,
the LD19 dropouts and the same rule judge as the Gazebo rounds. ``simulate`` below is a copy of
``open_lab.run_round`` (same random draws in the same order, so the results are identical; ``check_equivalence``
asserts it against ``run_round``) that also records, at 20 Hz, what the player needs.

POSE CONVENTIONS (what x, y, yaw, steer refer to)
  * x east, y north, in metres; the island is centred at (0, 0); the standard ``square100`` field has outer walls at
    +-1.5 m and the island walls at +-0.5 m. yaw in radians, 0 = facing +x (east), counter-clockwise positive.
  * (x, y) is the pose of ``base_link``: the point midway between the axles (the URDF origin; the car mesh needs NO offset
    if the player places it at base_link). The lab turns the car about its REAR axle (bicycle model) and then reports
    base_link, so base_link slides sideways in curves; this is what Gazebo's ``/odom_gt`` also reports.
  * The LiDAR sits LIDAR_OFFSET_X = 0.128 m ahead of base_link along the car's axis. ``scans[].ranges[i]`` is the
    range at bearing  yaw - 110 deg + i * (220 deg / 305)  (i = 0..305, counter-clockwise, 0 deg = car's nose) from that
    point; null = no return (dropout, out of range or beyond 12 m). It is the scan the strategy really used
    (8 mm noise, 4.3 % dropouts).
  * yaw is continuous (not wrapped to +-pi: the car goes round three times, so it reaches +-19 rad); use it modulo 2 pi.
  * steer = real wheel angle (rad) of the front axle after the servo's dead time, lag and rate limit; positive = left.
  * Start pose of the lab: (0, -1.15), yaw 0 (ccw) or pi (cw): middle of the south corridor, start section 0. Time 0 = the
    Start (the first scan). Frames every 0.05 s.

Run (RoboStack env, in one shell call):
    source tools/macos_env.sh ~/Projects/WRO/.mamba/jazzy >/dev/null 2>&1
    source install/setup.zsh 2>/dev/null
    PYTHONPATH=src/my_open:$PYTHONPATH python3 course/maintainers/replays/steps_6_9.py [--out DIR] [--png DIR]
"""
import argparse
import json
import math
import tempfile
from pathlib import Path

import numpy as np

from robotek_common import vehicle
from robotek_course.base import StrategyBase
from robotek_course.minisim import STEER_LIMIT, WHEELBASE, Scene, real_speed, wrap
from robotek_evaluation import open_lab as L
from robotek_evaluation.footprint import collision_polygons
from robotek_evaluation.rule_judge import footprint_section, judge_open

from my_open.arc import arc_steer
from my_open.heading import GyroHeading
from my_open.lane import GAP, lane_steer
from my_open.perception import perceive
from my_open.strategy import Strategy
from my_open.turn_gate import ISLAND_OPEN, TurnGate

OUT = Path('/Users/damian/Projects/Portfolio-Astro/public/learn/assets/replays')
FRAME_DT = 0.05
COLOR_MAIN, COLOR_GHOST = '#e8a33d', '#7aa7d9'
WORLD = L.FIELDS / 'square100.sdf'


# ------------------------------------------------------------------ step 6 strategy
class Step6Strategy(StrategyBase):
    """drive_step6.py as a lab strategy: lane + corner + TurnGate, direction fixed ccw, it never stops."""

    CRUISE = 1.28
    CALIBRATE_SECONDS = 0.6
    TURN_TOLERANCE = 0.12

    def __init__(self):
        super().__init__()
        self.heading = GyroHeading()
        self.samples = []
        self.time = 0.0
        self.goal = self.ref = 0.0
        self.arc = False
        self.corners = 0
        self.gate = TurnGate()
        self.scene = Scene()
        self.in_setup = True

    def update(self, dt):
        if not self.started:
            return 0.0, 0.0
        if self.new_scan:
            self.scene = perceive(self.scan, forward_yaw=wrap(self.goal - self.heading.yaw), sign=1)
        self.time += dt
        if self.time < self.CALIBRATE_SECONDS:
            self.samples.append(self.gyro)
            return 0.0, 0.0
        if self.samples:
            self.heading.calibrate(self.samples)
            self.samples = []
        self.in_setup = False
        yaw = self.heading.update(self.gyro, dt)
        self.yaw = yaw
        gyro = self.gyro - self.heading.bias
        self.travelled = max(0.0, self.travelled)
        self.since_corner = getattr(self, 'since_corner', 0.0) + self.travelled
        inner = self.scene.left
        if self.gate.update(self.scene, inner, self.since_corner, self.arc):
            self.goal = wrap(self.goal + math.pi / 2)
            self.arc = True
            self.gate.corner_started()
        if self.arc:
            rate = real_speed(self.CRUISE) * math.tan(STEER_LIMIT) / WHEELBASE
            steer, self.ref = arc_steer(self.goal, self.ref, yaw, gyro, rate, dt, sign=1)
            if abs(wrap(self.goal - yaw)) <= self.TURN_TOLERANCE:
                self.arc, self.ref, self.since_corner = False, self.goal, 0.0
                self.corners += 1
            return self.CRUISE, steer
        island_open = math.isfinite(inner) and inner > ISLAND_OPEN
        keep = GAP if (not math.isfinite(inner) or island_open) else inner
        return self.CRUISE, lane_steer(self.goal, yaw, keep, sign=1)

    @property
    def label(self):
        if not self.started:
            return 'WAIT_START'
        return 'CALIBRATE' if self.in_setup else 'TURN' if self.arc else 'FOLLOW'

    @property
    def turns(self):
        return self.corners


# ------------------------------------------------------------------ the instrumented lab loop
def simulate(direction, factory, seed=0, tmax=180.0, judge=True, spawn=L.START, world=None):
    """A copy of ``open_lab.run_round`` that records the player's data. Returns a dict."""
    rng = np.random.default_rng(seed)
    world = world or L.World(WORLD)
    strategy = factory()
    actuators = L.Actuators({**L.actuator_parameters()})
    sign = 1.0 if direction == 'ccw' else -1.0
    yaw = 0.0 if direction == 'ccw' else math.pi
    back = -vehicle.REAR_AXLE_X
    xr, yr = spawn[0] - back * math.cos(yaw), spawn[1] - back * math.sin(yaw)
    x, y = spawn
    angle_min = -L.SCAN_HALF_ANGLE
    increment = -2.0 * angle_min / (L.SCAN_RAYS - 1)
    beams = angle_min + increment * np.arange(L.SCAN_RAYS)
    bias = L.GYRO_BIAS + rng.normal(0.0, 0.0005)
    dt = 1.0 / L.PHYSICS_HZ
    rows, events = [], []
    t_start = None
    camera_time = None
    contact = None
    clearance = {'outer_wall': math.inf, 'inner_wall': math.inf}
    steer_max = 0.0
    yaw_rate = 0.0
    stopped_at = None
    pending = []
    # recorded for the player
    frames, scans, scenes, label_log, speed_log, dir_log = [], [], [], [], [], []
    last_scan = None
    cmd_speed = 0.0
    v_out = 0.0
    for tick in range(int(tmax / dt)):
        now = tick * dt
        pose = (x, y, yaw, actuators.steer)          # pose at the start of this tick: what the scan below sees
        if tick % L.SCAN_EVERY == 0:
            ranges = world.raycast(x + vehicle.LIDAR_OFFSET_X * math.cos(yaw),
                                   y + vehicle.LIDAR_OFFSET_X * math.sin(yaw), beams + yaw)
            ranges = ranges + rng.normal(0.0, L.SCAN_NOISE, L.SCAN_RAYS)
            ranges[(ranges < L.SCAN_MIN_RANGE) | ~np.isfinite(ranges)] = math.inf
            for copy_index in range(L.SCAN_COPIES):
                copy = ranges.copy()
                copy[rng.random(L.SCAN_RAYS) < L.SCAN_DROPOUT] = math.inf
                if copy_index == 0:
                    last_scan = (round(now, 2), [None if not math.isfinite(r) or r > 12.0 else round(float(r), 3)
                                                 for r in copy])
                if strategy.on_scan(L.make_scan(list(copy), now, angle_min, increment)):
                    t_start = now
                    events.append({'event': 'start', 'sim_time': now, 'wall_time': now})
        if tick % L.IMU_EVERY == 0:
            strategy.on_imu(now, yaw_rate + bias + rng.normal(0.0, L.GYRO_NOISE))
        if t_start is not None and now >= t_start + L.CAMERA_DELAY and \
                (camera_time is None or now - camera_time >= L.CAMERA_PERIOD - 1e-9):
            camera_time = now
            strategy.on_camera(int(sign))
        if tick % L.CONTROL_EVERY == 0:
            age = math.inf if camera_time is None else now - camera_time
            speed, steer = strategy.control(dt * L.CONTROL_EVERY, age)
            cmd_speed = speed
            pending.append((now + 0.0, speed, steer))
            key = (strategy.label, strategy.turns)
            if not label_log or label_log[-1][1:] != key:
                label_log.append((round(now, 2),) + key)
            mission = getattr(strategy, 'mission', None)
            dir_log.append((now, mission.direction if mission else None))
            speed_log.append((now, speed))
        while pending and pending[0][0] <= now + 1e-9:
            actuators.command(now, *pending.pop(0)[1:])
        v_out, commanded_rate = actuators.step(now, dt)
        yaw_rate += commanded_rate - yaw_rate
        yaw += yaw_rate * dt
        xr += v_out * math.cos(yaw) * dt
        yr += v_out * math.sin(yaw) * dt
        x, y = xr + back * math.cos(yaw), yr + back * math.sin(yaw)
        steer_max = max(steer_max, abs(actuators.steer))
        if tick % 5 == 0:                              # 20 Hz frames: pose at the start of the tick
            frames.append((round(now, 2), pose[0], pose[1], pose[2], pose[3], v_out))
            sc = getattr(strategy, 'scene', None)
            scenes.append((round(now, 2), sc))
            if last_scan is not None and last_scan[0] == round(now, 2):
                scans.append(last_scan)
        if tick % L.CONTROL_EVERY == 0:
            clearance = {k: min(clearance[k], v) for k, v in world.clearances(L.outline(x, y, yaw)).items()}
            if contact is None and min(clearance.values()) <= 0.0:
                contact = now
        if tick % L.TELEMETRY_EVERY == 0:
            rows.append(L._row(now, strategy.label, x, y, yaw, actuators.steer))
        if contact is not None:
            break
        if strategy.done and stopped_at is None:
            stopped_at = now
            events.append({'event': 'done', 'sim_time': now, 'wall_time': now})
        if stopped_at is not None and abs(v_out) < 1e-3 and abs(yaw_rate) < 1e-3 and now - stopped_at > 1.0:
            break
    last = rows[-1] if rows else None
    if stopped_at is not None and contact is None and last is not None:
        count = int(L.STOP_HOLD * L.PHYSICS_HZ / L.TELEMETRY_EVERY)
        rows.extend(L._row(last['sim_time'] + (i + 1) * L.TELEMETRY_EVERY / L.PHYSICS_HZ, last['state'],
                           x, y, yaw, actuators.steer) for i in range(count))
    out = dict(frames=frames, scans=scans, scenes=scenes, labels=label_log, speeds=speed_log, dirs=dir_log,
               rows=rows, events=events, contact=contact, strategy=strategy, steer_max=steer_max,
               t_end=round(now, 2), stopped_at=stopped_at)
    if judge:
        out['result'] = L._result(world, direction, strategy, rows, events, clearance, contact, steer_max, t_start)
        with tempfile.TemporaryDirectory() as tmp:
            out['judge'] = judge_open(Path(tmp), {'direction': direction, 'field_geometry': world.geometry}, rows, events)
        out['exits'] = section_exits(rows, direction)
    return out


def section_exits(rows, direction):
    """Times of the judge's section exits (a copy of the counting in judge_open): list of (exit number, sim_time)."""
    sign = 1 if direction == 'ccw' else -1
    current, reverse, exits, out = None, 0, 0, []
    for r in rows:
        section = footprint_section(sum(collision_polygons(r).values(), []))
        if current is None:
            current = section
            continue
        if section is None or section == current:
            continue
        if section == (current + sign) % 8:
            if reverse:
                reverse -= 1
            else:
                exits += 1
                out.append((exits, r['sim_time']))
                if exits == 25:
                    break
            current = section
        elif section == (current - sign) % 8:
            reverse += 1
            current = section
    return out


def check_equivalence():
    """simulate() must give exactly the result of open_lab.run_round (same draws, same judge)."""
    for direction, spec in (('ccw', 'ccw'), ('cw', 'auto')):
        a = simulate(direction, lambda: Strategy(spec))['result']
        b = L.run_round(WORLD, direction, lambda: Strategy(spec))
        assert a == b, (a, b)
    print('simulate() == open_lab.run_round on square100 ccw/cw: OK')


# ------------------------------------------------------------------ JSON writing
def r4(v):
    return round(float(v), 4)


def finite(v, nd=3):
    return None if v is None or not math.isfinite(v) else round(float(v), nd)


def track(run, ident, label, color, ghost=False, until=None):
    frames = [[t, r4(x), r4(y), r4(yaw), r4(st)] for t, x, y, yaw, st, _ in run['frames']
              if until is None or t <= until]
    out = {'id': ident, 'label': label, 'color': color, 'frames': frames}
    if ghost:
        out['ghost'] = True
    return out


def scene_list(run, until, every=0.1):
    out, k = [], round(every / FRAME_DT)
    for i, (t, sc) in enumerate(run['scenes']):
        if t > until or i % k or sc is None:
            continue
        out.append({'t': t, 'left': finite(sc.left), 'right': finite(sc.right), 'front': finite(sc.front_axis),
                    'island_end': finite(sc.island_end)})
    return out


def scan_list(run, until, every, windows=None):
    out = []
    stride = round(every / 0.1)
    for i, (t, ranges) in enumerate(run['scans']):
        if t > until or i % stride:
            continue
        if windows is not None and not any(a <= t <= b for a, b in windows):
            continue
        out.append({'t': t, 'track': 'main', 'ranges': ranges})
    return out


def label_list(run, until):
    # 'corners' is the number of corners finished; one entry per change of (state, corners)
    return [{'t': t, 'state': s, 'corners': c} for t, s, c in run['labels'] if t <= until]


def fmt(seconds):
    return ('%.2f' % seconds).replace('.', ',')


def write(name, doc):
    path = OUT / name
    text = json.dumps(doc, separators=(',', ':'), ensure_ascii=False)
    path.write_text(text)
    return path, len(text.encode())


def corner_starts(run):
    """(time, corner number) when each corner begins (label TURN)."""
    out, seen = [], set()
    for t, s, c in run['labels']:
        if s == 'TURN' and c + 1 not in seen:
            seen.add(c + 1)
            out.append((t, c + 1))
    return out


def stop_time(run):
    """First time after the FINISH label at which the car is practically at rest (< 5 cm/s; the speed model's tail is slow)."""
    t_finish = next(t for t, s, c in run['labels'] if s == 'FINISH')
    for t, _, _, _, _, v in run['frames']:
        if t >= t_finish and abs(v) < 0.05:
            return t
    return None


def brake_time(run):
    """When the last corner's slow-down begins: the first commanded speed below cruise after 11 corners."""
    t_last = next(t for t, s, c in run['labels'] if c == 11)
    for t, v in run['speeds']:
        if t >= t_last and v < 1.28 - 1e-6:
            return round(t, 2)
    return None


def decision_time(run):
    prev = ''
    for t, d in run['dirs']:
        if d and not prev:
            return round(t, 2)
        prev = d
    return None


# ------------------------------------------------------------------ the four files
def step6():
    run = simulate('ccw', Step6Strategy, tmax=30.0, judge=False)
    assert run['contact'] is None, 'contact in step 6'
    corners = corner_starts(run)
    until = run['t_end']
    events = [{'t': 0.0, 'text': 'Start: el coche calibra el giroscopio, quieto'}]
    for t, n in corners:
        events.append({'t': t, 'text': 'Esquina %d. La puerta de giro dispara: pared cerca e isla terminada' % n})
    windows = [(t - 1.2, t + 1.5) for t, n in corners[:2]]
    doc = {'version': 1, 'step': 6, 'title': 'Cuándo girar: la puerta de giro',
           'caption': 'El coche gira esquina tras esquina sin parar: la puerta dispara solo cuando la pared está cerca y la isla terminó.',
           'field': 'square100', 'direction': 'ccw', 'duration': until,
           'tracks': [track(run, 'main', 'Tu estrategia', COLOR_MAIN)], 'trail': True,
           'scans': scan_list(run, until, 0.3, windows), 'scene': scene_list(run, until, 0.1),
           'labels': label_list(run, until), 'sections': False, 'events': events}
    return doc, dict(run=run, corners=len(corners), result=None)


def step7():
    run = simulate('ccw', lambda: Strategy('ccw'))
    res, judge = run['result'], run['judge']
    until = round(run['stopped_at'] + 1.0, 2)
    cum = [lap['cumulative_sim_seconds'] for lap in judge['lap_splits']]
    t_brake, t_stop = brake_time(run), stop_time(run)
    events = [{'t': 0.0, 'text': 'Start: calibra el giroscopio, quieto 0,3 s'}]
    for i, t in enumerate(cum, 1):
        events.append({'t': round(t, 2), 'text': 'Vuelta %d/3 completada (%s s desde Start)' % (i, fmt(t))})
    events.append({'t': t_brake, 'text': 'Frena en la última esquina para poder parar en la sección de salida'})
    if t_stop is not None:
        events.append({'t': t_stop, 'text': 'Parada en la sección de salida'})
    events.append({'t': round(run['stopped_at'], 2), 'text': 'DONE: 12 esquinas, ronda de %s s' % fmt(res['time'])})
    events.sort(key=lambda e: e['t'])
    doc = {'version': 1, 'step': 7, 'title': 'La misión: tres vueltas y parada',
           'caption': 'Tres vueltas (12 esquinas), frenada en la última y parada en la sección de salida: %d de 30 puntos en %s s.'
                      % (res['points'], fmt(res['time'])),
           'field': 'square100', 'direction': 'ccw', 'duration': until,
           'tracks': [track(run, 'main', 'Tu estrategia', COLOR_MAIN)], 'trail': True,
           'scans': scan_list(run, until, 0.4), 'scene': scene_list(run, until, 0.1),
           'labels': label_list(run, until), 'sections': True, 'events': events}
    return doc, dict(run=run, result=res)


def step8():
    """Main: ccw round decided by the camera ('auto'); ghost: the cw round, also decided by the camera."""
    run = simulate('ccw', lambda: Strategy('auto'))
    ghost = simulate('cw', lambda: Strategy('auto'))
    res, gres = run['result'], ghost['result']
    until = round(run['stopped_at'] + 1.0, 2)
    t_dec, g_dec = decision_time(run), decision_time(ghost)
    events = [{'t': 0.0, 'text': 'Start: calibra el giroscopio, quieto'},
              {'t': t_dec, 'text': 'La cámara decide: azul más larga, sentido antihorario '
                                   '(fantasma: naranja más larga, sentido horario)'},
              {'t': round(run['stopped_at'], 2),
               'text': 'DONE: %d de 30 puntos en %s s (fantasma, sentido horario: %d en %s s)'
                       % (res['points'], fmt(res['time']), gres['points'], fmt(gres['time']))}]
    doc = {'version': 1, 'step': 8, 'title': 'La dirección: la cámara decide el sentido',
           'caption': 'La misma estrategia decide sola el sentido con la cámara: antihorario aquí y horario en el fantasma, los dos con 30 puntos.',
           'field': 'square100', 'direction': 'ccw', 'duration': until,
           'tracks': [track(run, 'main', 'Sentido antihorario (azul más larga)', COLOR_MAIN),
                      track(ghost, 'ghost', 'Sentido horario (naranja más larga)', COLOR_GHOST, ghost=True, until=until)],
           'trail': True, 'scene': scene_list(run, until, 0.1),
           'labels': label_list(run, until), 'sections': True, 'events': events}
    return doc, dict(run=run, ghost=ghost, result=res, ghost_result=gres, decision=(t_dec, g_dec))


def step9():
    run = simulate('cw', lambda: Strategy('auto'))
    res, judge, exits = run['result'], run['judge'], run['exits']
    until = round(run['stopped_at'] + 1.0, 2)
    exit_time = dict(exits)
    t_dec = decision_time(run)
    events = [{'t': 0.0, 'text': 'Start: el juez empieza a contar el tiempo'},
              {'t': t_dec, 'text': 'La cámara decide: naranja más larga, sentido horario'},
              {'t': round(exit_time[1], 2), 'text': 'Sale de la sección de salida: el primer punto de sección'}]
    for lap in judge['lap_splits']:
        n = lap['lap']
        text = 'Vuelta %d: %d secciones + %d de vuelta = %d puntos (%s s desde Start)' % (
            n, 8 * n, n, 9 * n, fmt(lap['cumulative_sim_seconds']))
        if n == 3:        # the 24th section exit and the third lap are the same instant for the judge
            text = 'Sección 24 de 24 y vuelta 3: 24 + 3 = 27 puntos (%s s desde Start)' % fmt(lap['cumulative_sim_seconds'])
        events.append({'t': round(lap['cumulative_sim_seconds'], 2), 'text': text})
    events.append({'t': round(run['stopped_at'], 2),
                   'text': 'Parada en la sección de salida: +3, total %d de 30 en %s s desde Start (el juez la observa 15 s)'
                           % (res['points'], fmt(res['time']))})
    events.sort(key=lambda e: e['t'])
    doc = {'version': 1, 'step': 9, 'title': 'Puntuar y medir: 24 secciones, 3 vueltas, parada',
           'caption': 'Replay de una ronda del laboratorio, %d de 30' % res['points'],
           'field': 'square100', 'direction': 'cw', 'duration': until,
           'tracks': [track(run, 'main', 'Ronda del laboratorio (sentido horario)', COLOR_MAIN)], 'trail': True,
           'scene': scene_list(run, until, 0.1),
           'labels': label_list(run, until), 'sections': True, 'events': events}
    return doc, dict(run=run, result=res, judge=judge, exits=exits)


# ------------------------------------------------------------------ checks and pictures
def validate(doc, path, size):
    assert json.loads(Path(path).read_text())['version'] == 1
    for tr in doc['tracks']:
        ts = [f[0] for f in tr['frames']]
        assert all(abs((b - a) - FRAME_DT) < 1e-6 for a, b in zip(ts, ts[1:])), 'dt not fixed'
        assert ts[0] == 0.0
        assert abs(ts[-1] - doc['duration']) < FRAME_DT + 1e-9, (ts[-1], doc['duration'])
    for key in ('scans', 'scene', 'labels', 'events'):
        ts = [e['t'] for e in doc.get(key, [])]
        assert ts == sorted(ts), key + ' not monotonic'
        assert all(0 <= t <= doc['duration'] + 1e-9 for t in ts), key + ' beyond duration'
    for s in doc.get('scans', []):
        assert len(s['ranges']) == 306
    assert size < 500_000, 'too big: %d' % size


def png(doc, extra, folder):
    """Sanity picture (OpenCV; matplotlib is not in the RoboStack env): field, 8 mat sections and the path coloured by time."""
    import cv2
    scale, size = 220, 800
    img = np.full((size, size, 3), 255, np.uint8)

    def px(x, y):
        return int(round(size / 2 + x * scale)), int(round(size / 2 - y * scale))

    cv2.rectangle(img, px(-0.5, 0.5), px(0.5, -0.5), (170, 170, 170), -1)
    cv2.rectangle(img, px(-1.52, 1.52), px(1.52, -1.52), (0, 0, 0), 3)
    for v in (-0.5, 0.5):                          # the 8 mat sections of rule_judge.mat_section
        cv2.line(img, px(v, -1.5), px(v, -0.5), (0, 150, 0), 1)
        cv2.line(img, px(v, 0.5), px(v, 1.5), (0, 150, 0), 1)
        cv2.line(img, px(-1.5, v), px(-0.5, v), (0, 150, 0), 1)
        cv2.line(img, px(0.5, v), px(1.5, v), (0, 150, 0), 1)
    cv2.line(img, px(0, -0.5), px(0, 0.5), (200, 200, 200), 1)
    cv2.line(img, px(-0.5, 0), px(-0.5, 0), (0, 150, 0), 1)
    for tr in reversed(doc['tracks']):
        n = len(tr['frames'])
        for i, (t, x, y, yaw, st) in enumerate(tr['frames']):
            f = i / max(1, n - 1)
            color = (int(255 * (1 - f)), 80, int(255 * f)) if not tr.get('ghost') else (230, 200, 120)
            cv2.circle(img, px(x, y), 2, color, -1)
        t, x, y, yaw, st = tr['frames'][0]
        cv2.circle(img, px(x, y), 7, (0, 0, 255), 2)
        cv2.arrowedLine(img, px(x, y), px(x + 0.2 * math.cos(yaw), y + 0.2 * math.sin(yaw)), (0, 0, 255), 2)
        t, x, y, yaw, st = tr['frames'][-1]
        cv2.rectangle(img, (px(x, y)[0] - 5, px(x, y)[1] - 5), (px(x, y)[0] + 5, px(x, y)[1] + 5), (0, 0, 0), 2)
    cv2.putText(img, 'step %d %s %.2f s frames=%d (red=start blue=end)' % (
        doc['step'], doc['direction'], doc['duration'], len(doc['tracks'][0]['frames'])), (12, 24),
        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
    folder.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(folder / ('step-%d.png' % doc['step'])), img)


def main():
    global OUT
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, default=OUT)
    parser.add_argument('--png', type=Path, default=Path('/tmp/replays_check2'))
    args = parser.parse_args()
    OUT = args.out
    OUT.mkdir(parents=True, exist_ok=True)
    check_equivalence()
    rows = []
    for fn in (step6, step7, step8, step9):
        doc, info = fn()
        path, size = write('step-%d.json' % doc['step'], doc)
        validate(doc, path, size)
        png(doc, info, args.png)
        res = info.get('result')
        rows.append((doc['step'], doc['direction'], size, doc['duration'], len(doc['tracks'][0]['frames']),
                     len(doc.get('scans', [])), len(doc.get('scene', [])),
                     ('%d pts, %.2f s, valid=%s' % (res['points'], res['time'], res['valid'])) if res else 'n/a (no judge)'))
        print('step', doc['step'], 'labels:', [(l['t'], l['state'], l['corners']) for l in doc['labels']][:40])
        print('   events:', [(e['t'], e['text'][:60]) for e in doc['events']])
        if 'decision' in info:
            print('   camera decision (main, ghost):', info['decision'])
        if res:
            print('   judge:', {k: res[k] for k in ('points', 'time', 'valid', 'failed', 'margin', 'min_clearance', 'laps')})
    print('\nstep dir  bytes  duration frames scans scene result')
    for r in rows:
        print('%-4d %-4s %-6d %-8.2f %-6d %-5d %-5d %s' % r)


if __name__ == '__main__':
    main()
