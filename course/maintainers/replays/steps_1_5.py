#!/usr/bin/env python3
"""Replays (trajectories) for steps 1 to 5 of the guide, from the Python mini simulator. No Gazebo.

    source tools/macos_env.sh <env>; export PYTHONPATH=src/my_open:src/robotek_course:src/robotek_common:src/robotek_evaluation
    python3 course/maintainers/replays/steps_1_5.py [OUTPUT_DIR]

Writes step-1.json .. step-5.json (the format the guide's replay player reads) with the REFERENCE modules of
``my_open`` (model, heading, perception, lane, arc) in the loop, driven exactly like course/snapshots/drive_stepN.py
(50 Hz control, scans at 10 Hz, 0.6 s of gyroscope calibration from step 2 on, fixed random seeds).

POSE CONVENTION (for the player). x, y, yaw are the pose of ``base_link`` in the field frame (x east, y north, yaw
counter-clockwise from +x, island centred at (0, 0)). In the mini simulator base_link is the point that the bicycle
model moves (the car's centre in the guide's drawings), and it is the origin the simulator uses for everything else:
    * LiDAR origin: 0.128 m ahead of (x, y) along yaw  (scan rays: yaw - 110 deg .. yaw + 110 deg, 306 rays);
    * outline: 0.1477 m ahead of (x, y), 0.0852 m behind, half width 0.074 m (robotek_common/vehicle.py);
    * axles (for drawing the wheels): +-0.0587 m from (x, y), wheelbase 0.1174 m.
So NO offset has to be applied: draw the car with base_link at (x, y). ``steer`` is the wheel angle really reached
(after the servo's dead time and lag), positive = left, in rad. ``yaw`` is continuous (unwrapped): it can pass pi.
``scene.island_end`` is in the car's frame (m ahead of base_link, as perceive() returns it). Scan ranges are measured
from the LiDAR origin; ``null`` = no return. A scan at time t belongs to the pose of the track at time t.
"""
import json
import math
import os
import sys

import numpy as np

from my_open.arc import arc_steer
from my_open.heading import GyroHeading, heading_steer
from my_open.lane import GAP, lane_steer
from my_open.model import turn_radius
from my_open.perception import perceive
from robotek_common.vehicle import WHEELBASE
from robotek_course.minisim import (SCAN_HALF, SCAN_RAYS, STEER_LIMIT, Field, MiniCar, WALLS, real_speed, wrap)

DT = 0.02                 # s: control and simulation tick (50 Hz, as the nodes)
FRAME_DT = 0.05           # s: spacing of the frames in the JSON
SCAN_EVERY = 5            # ticks between LiDAR scans (10 Hz, like the LD19)
CALIBRATE_SECONDS = 0.6
HONEST = 'Replay del mini simulador de Python (mismo código que el nodo), no una grabación de Gazebo.'
MAIN_COLOR, GHOST_COLOR = '#e8a33d', '#ff7a6b'
OUT_DEFAULT = 'replays-out'            # the guide's site copies these files to public/learn/assets/replays/

# The fields. square100 = minisim.WALLS. long-island = the world of course/verify_step.py lane_world(): the island
# (x from -0.5 to 9.0, 1 m wide) with the outer walls 1 m around it and the east wall at x = 10.0.
LONG_ISLAND_WALLS = (
    (-1.52, 0.0, 0.04, 3.08),        # outer west (unchanged)
    (10.02, 0.0, 0.04, 3.08),        # outer east
    (4.24, 1.52, 11.64, 0.04),       # outer north
    (4.24, -1.52, 11.64, 0.04),      # outer south
    (-0.48, 0.0, 0.04, 1.00),        # island west end (unchanged)
    (8.98, 0.0, 0.04, 1.00),         # island east end
    (4.25, 0.48, 9.50, 0.04),        # island north face
    (4.25, -0.48, 9.50, 0.04),       # island south face (face at y = -0.5)
)


def es(value, digits=2):
    """Number with a decimal comma, as in the guide's Spanish text."""
    return ('%.*f' % (digits, value)).replace('.', ',')


def r(value, digits):
    return None if value is None or not math.isfinite(value) else round(value, digits)


class Log:
    """What a run leaves: one pose per tick, scans, scenes and notes, ready to be resampled at 0.05 s."""

    def __init__(self):
        self.ticks = []          # (t, x, y, yaw, steer, speed)
        self.scans = []          # (t, ranges)
        self.scenes = []         # (t, Scene)
        self.labels = []         # (t, state, corners)
        self._yaw0 = 0.0

    def pose(self, car):
        yaw = car.yaw
        if self.ticks:                                  # keep the yaw continuous
            previous = self.ticks[-1][3]
            yaw = previous + wrap(car.yaw - previous)
        self.ticks.append((car.time, car.x, car.y, yaw, car.steer, car.speed))

    def label(self, t, state, corners):
        if not self.labels or (self.labels[-1][1], self.labels[-1][2]) != (state, corners):
            self.labels.append((t, state, corners))

    def frames(self, duration):
        """Frames at fixed FRAME_DT from t = 0 to ``duration``; the car stays where it ended after its last tick."""
        times = [row[0] for row in self.ticks]
        columns = list(zip(*self.ticks))
        out = []
        for i in range(int(round(duration / FRAME_DT)) + 1):
            t = i * FRAME_DT
            row = [t]
            for c in range(1, 5):
                row.append(float(np.interp(t, times, columns[c])))
            out.append([round(row[0], 2)] + [round(v, 4) for v in row[1:]])
        return out

    def at(self, t):
        """The tick row nearest to time t."""
        times = [row[0] for row in self.ticks]
        return self.ticks[min(range(len(times)), key=lambda i: abs(times[i] - t))]


def scan_ranges(scan):
    return [r(v, 3) for v in scan.ranges]


# ---------------------------------------------------------------------------------------------- step 1
def run_circle(steer, speed_command, seconds, realistic, seed=1):
    car = MiniCar(0.0, 0.0, 0.0, realistic=realistic, seed=seed)
    log = Log()
    log.pose(car)
    for _ in range(int(round(seconds / DT))):
        car.step(speed_command, steer, DT)
        log.pose(car)
    return log


def fit_circle(points):
    """Least-squares circle through points: returns (cx, cy, radius)."""
    a = np.array([[2 * x, 2 * y, 1.0] for x, y in points])
    b = np.array([x * x + y * y for x, y in points])
    cx, cy, c = np.linalg.lstsq(a, b, rcond=None)[0]
    return cx, cy, math.sqrt(c + cx * cx + cy * cy)


def step1():
    steer, speed, seconds = 0.25, 0.6, 12.0
    main = run_circle(steer, speed, seconds, True)
    ghost_real_speed = real_speed(speed)
    ghost = run_circle(steer, ghost_real_speed, seconds, False)     # ideal car: instant servo, the speed the car really reaches
    expected = turn_radius(steer)
    fit = [(row[1], row[2]) for row in main.ticks if row[0] >= 1.5]
    cx, cy, radius = fit_circle(fit)
    lap = 2 * math.pi * expected / ghost_real_speed
    lag = max(math.hypot(a[1] - b[1], a[2] - b[2]) for a, b in zip(main.ticks, ghost.ticks))
    reach = next(row[0] for row in main.ticks if row[4] >= 0.95 * steer)
    events = [
        {'t': 0.0, 'text': 'Volante fijo a %s rad, velocidad 0,42 m/s' % es(steer)},
        {'t': round(reach / 2, 2), 'text': 'El servo tarda %s s en llegar al ángulo pedido' % es(reach)},
        {'t': 2.5, 'text': 'Radio esperado: %s m (distancia entre ejes / tan %s)' % (es(expected, 3), es(steer))},
        {'t': 5.0, 'text': 'Radio medido en este replay: %s m' % es(radius, 3)},
        {'t': round(lap + reach / 2, 2), 'text': 'Una vuelta completa en unos %s s' % es(lap, 1)},
        {'t': 10.5, 'text': 'El modelo ideal (sin servo) va hasta %s cm por delante' % es(lag * 100, 0)},
    ]
    return {
        'version': 1, 'step': 1, 'title': 'Paso 1: el círculo con el volante fijo',
        'caption': 'Con el volante a %s rad el coche dibuja un círculo de radio %s m, como predice el modelo de bicicleta. %s'
                   % (es(steer), es(expected, 3), HONEST),
        'field': 'empty', 'direction': 'ccw', 'duration': seconds,
        'tracks': [
            {'id': 'main', 'label': 'Coche con servo real', 'color': MAIN_COLOR, 'frames': main.frames(seconds)},
            {'id': 'ghost', 'label': 'Modelo ideal (sin servo)', 'color': GHOST_COLOR, 'frames': ghost.frames(seconds)},
        ],
        'trail': True, 'scans': [], 'scene': [], 'labels': [], 'sections': False, 'events': events,
    }, {'radius_fit': radius, 'centre': (cx, cy), 'expected': expected, 'lag_max_m': lag, 'lap_s': lap, 'reach': reach}


# ---------------------------------------------------------------------------------------------- step 2
def run_straight(calibrate, seconds=10.0, speed_command=0.6, seed=2):
    """drive_step2.py: stand still CALIBRATE_SECONDS, then hold heading 0. Returns the log and the true yaw at the end."""
    car = MiniCar(0.0, 0.0, 0.0, realistic=True, seed=seed)
    heading, samples = GyroHeading(), []
    log = Log()
    log.pose(car)
    total = CALIBRATE_SECONDS + seconds
    time = 0.0
    for _ in range(int(round(total / DT))):
        gyro = car.gyro()
        speed, steer = 0.0, 0.0
        time += DT
        if time < CALIBRATE_SECONDS:
            samples.append(gyro)
        else:
            if samples:
                if calibrate:
                    heading.calibrate(samples)
                samples = []
            yaw = heading.update(gyro, DT)
            speed, steer = speed_command, heading_steer(0.0, yaw)
        car.step(speed, steer, DT)
        log.pose(car)
    return log, heading


def step2():
    main, h_main = run_straight(True)
    ghost, h_ghost = run_straight(False)
    duration = round(CALIBRATE_SECONDS + 10.0, 2)
    end_main, end_ghost = main.ticks[-1], ghost.ticks[-1]
    events = [
        {'t': 0.0, 'text': 'Calibra %s s quieto' % es(CALIBRATE_SECONDS, 1)},
        {'t': 0.7, 'text': 'Sesgo medido: %s rad/s' % es(h_main.bias, 4)},
        {'t': 5.0, 'text': 'El sesgo se acumula'},
        {'t': round(duration - 0.4, 2), 'text': 'Sin calibrar el rumbo real acaba %s rad torcido; calibrado, %s rad'
                                                 % (es(end_ghost[3], 3), es(end_main[3], 3))},
    ]
    return {
        'version': 1, 'step': 2, 'title': 'Paso 2: el rumbo con el giroscopio',
        'caption': 'Sin calibrar el giroscopio el coche cree ir recto pero se tuerce %s rad en 10 s; calibrado sigue recto. %s'
                   % (es(abs(end_ghost[3]), 2), HONEST),
        'field': 'empty', 'direction': 'ccw', 'duration': duration,
        'tracks': [
            {'id': 'main', 'label': 'Con calibración', 'color': MAIN_COLOR, 'frames': main.frames(duration)},
            {'id': 'ghost', 'label': 'Sin calibrar', 'color': GHOST_COLOR, 'frames': ghost.frames(duration)},
        ],
        'trail': True, 'scans': [], 'scene': [], 'labels': [], 'sections': False, 'events': events,
    }, {'yaw_main': end_main[3], 'yaw_ghost': end_ghost[3], 'bias': h_main.bias}


# ---------------------------------------------------------------------------------------------- step 3
def record_scan(log, field, car, tick, heading_yaw, goal, sign=1):
    """A scan every SCAN_EVERY ticks, perceived like on_scan() does; returns the Scene (or None between scans)."""
    if tick % SCAN_EVERY:
        return None
    scan = field.scan(car.x, car.y, car.yaw)
    scene = perceive(scan, forward_yaw=wrap(goal - heading_yaw), sign=sign)
    log.scans.append((car.time, scan_ranges(scan)))
    log.scenes.append((car.time, scene))
    return scene


def scene_json(log):
    return [{'t': round(t, 2), 'left': r(s.left, 3), 'right': r(s.right, 3), 'front': r(s.front_axis, 3),
             'island_end': r(s.island_end, 3)} for t, s in log.scenes]


def scans_json(log, every_s):
    step = int(round(every_s / (SCAN_EVERY * DT)))
    return [{'t': round(t, 2), 'track': 'main', 'ranges': ranges} for i, (t, ranges) in enumerate(log.scans) if i % step == 0]


def run_step3(stop_front=0.5, speed_command=0.6, start=(0.0, -1.0, 0.0), tail=0.8, max_seconds=12.0, seed=3):
    """drive_step3.py: hold heading 0 until the wall ahead is nearer than ``stop_front``, then stop."""
    field = Field(WALLS, seed=seed)
    car = MiniCar(*start, realistic=True, seed=seed)
    heading, samples = GyroHeading(), []
    log = Log()
    log.pose(car)
    scene, stopped, time, quiet = None, False, 0.0, 0.0
    for tick in range(int(round(max_seconds / DT))):
        got = record_scan(log, field, car, tick, heading.yaw, 0.0)
        scene = got or scene
        gyro = car.gyro()
        speed, steer = 0.0, 0.0
        time += DT
        if time < CALIBRATE_SECONDS:
            samples.append(gyro)
        else:
            if samples:
                heading.calibrate(samples)
                samples = []
            yaw = heading.update(gyro, DT)
            stopped = stopped or (scene is not None and scene.front_axis < stop_front)
            if not stopped:
                speed, steer = speed_command, heading_steer(0.0, yaw)
        car.step(speed, steer, DT)
        log.pose(car)
        quiet = quiet + DT if stopped and car.speed < 0.005 else 0.0
        if quiet >= tail:
            break
    return log, field


def step3():
    log, field = run_step3()
    duration = math.ceil(log.ticks[-1][0] * 20) / 20
    scenes = log.scenes
    first = scenes[0][1]
    jump = next(((t, s) for t, s in scenes if math.isfinite(s.left) and s.left > 1.0), None)
    stop_scene = next(((t, s) for t, s in scenes if s.front_axis < 0.5), scenes[-1])
    events = [
        {'t': 0.0, 'text': 'Empieza: left %s m (la isla), right %s m (el muro), front %s m'
                           % (es(first.left), es(first.right), es(first.front_axis))},
        {'t': 0.3, 'text': 'Quieto %s s: el giroscopio se calibra mientras el LiDAR ya mide' % es(CALIBRATE_SECONDS, 1)},
    ]
    close = next(((t, s) for t, s in scenes if s.island_end < 0.25), None)
    events[0]['text'] += ', island_end %s m' % es(first.island_end)
    if close:
        events.append({'t': round(close[0], 2), 'text': 'island_end baja a %s m: la esquina de la isla se acerca' % es(close[1].island_end)})
    if jump:
        events.append({'t': round(jump[0], 2), 'text': 'left salta a %s m: ya no hay isla al lado' % es(jump[1].left)})
    events.append({'t': round(stop_scene[0], 2), 'text': 'front llega a %s m, por debajo de 0,5: el coche frena' % es(stop_scene[1].front_axis)})
    return {
        'version': 1, 'step': 3, 'title': 'Paso 3: de un barrido LiDAR a cuatro números',
        'caption': 'El coche recto por el pasillo sur: en cada barrido (rayos azules) perceive() saca left, right, front e island_end. %s' % HONEST,
        'field': 'square100', 'direction': 'ccw', 'duration': duration,
        'tracks': [{'id': 'main', 'label': 'Tu estrategia', 'color': MAIN_COLOR, 'frames': log.frames(duration)}],
        'trail': True, 'scans': scans_json(log, 0.2), 'scene': scene_json(log), 'labels': [], 'sections': False,
        'events': sorted(events, key=lambda e: e['t']),
    }, {'scenes': len(scenes), 'first': first, 'end': ticks_tail(log), 'stop_scene': stop_scene[1]}


def ticks_tail(log):
    return log.ticks[-1]


# ---------------------------------------------------------------------------------------------- step 4
def run_step4(speed_command=1.28, turn_front=1.19, start=(0.0, -1.0, 0.0), tail=0.8, max_seconds=40.0, seed=4):
    """drive_step4.py in the long-island field: lane following at 0.25 m, stop when the wall ahead is at 1.19 m."""
    field = Field(LONG_ISLAND_WALLS, seed=seed)
    car = MiniCar(*start, realistic=True, seed=seed)
    heading, samples = GyroHeading(), []
    log = Log()
    log.pose(car)
    scene, stopped, time, quiet = None, False, 0.0, 0.0
    gaps = []
    for tick in range(int(round(max_seconds / DT))):
        got = record_scan(log, field, car, tick, heading.yaw, 0.0)
        scene = got or scene
        gyro = car.gyro()
        speed, steer = 0.0, 0.0
        time += DT
        if time < CALIBRATE_SECONDS:
            samples.append(gyro)
        else:
            if samples:
                heading.calibrate(samples)
                samples = []
            yaw = heading.update(gyro, DT)
            stopped = stopped or (scene is not None and scene.front_axis <= turn_front)
            if not stopped:
                inner = scene.left if scene is not None and math.isfinite(scene.left) else GAP
                speed, steer = speed_command, lane_steer(0.0, yaw, inner, sign=1)
        car.step(speed, steer, DT)
        log.pose(car)
        gaps.append((car.time, field.island_gap(car.x, car.y, car.yaw)))
        quiet = quiet + DT if stopped and car.speed < 0.005 else 0.0
        if quiet >= tail:
            break
    return log, field, gaps, car


def step4():
    log, field, gaps, car = run_step4()
    duration = math.ceil(log.ticks[-1][0] * 20) / 20
    start_gap = field.island_gap(*log.ticks[0][1:4])
    # the gap when the stop order came (the last tick that drove) and the first time it is within 5 cm of 0.25 m
    moving = [(t, g) for (t, g), row in zip(gaps, log.ticks[1:]) if row[5] > 0.05]
    end_gap = moving[-1]
    near = next(((t, g) for t, g in gaps if g <= 0.30), None)
    half = next(((t, g) for t, g in gaps if g <= 0.375), None)
    min_gap = min(g for _, g in gaps)
    touched = any(field.touches(*row[1:4]) for row in log.ticks)
    events = [
        {'t': 0.0, 'text': 'Sale a %s m de la isla; la meta es %s m' % (es(start_gap), es(GAP))},
        {'t': round(half[0], 2), 'text': 'El hueco baja de 0,5 a 0,25 m: ya va por %s m' % es(half[1])} if half else None,
        {'t': round(near[0], 2), 'text': 'Sigue bajando: %s m, cada vez más despacio' % es(near[1])} if near else None,
        {'t': round(log.ticks[-1][0] - 0.4, 2), 'text': 'Al parar: hueco %s m%s' % (es(end_gap[1], 3), '' if not touched else ' (¡toca una pared!)')},
    ]
    events = [e for e in events if e]
    return {
        'version': 1, 'step': 4, 'title': 'Paso 4: seguir la pared de la isla',
        'caption': 'Con el volante empujado por el error del hueco, el coche se acerca a 0,25 m de la isla sin zigzaguear. %s' % HONEST,
        'field': 'long-island', 'direction': 'ccw', 'duration': duration,
        'tracks': [{'id': 'main', 'label': 'Tu estrategia', 'color': MAIN_COLOR, 'frames': log.frames(duration)}],
        'trail': True, 'scans': [], 'scene': scene_json(log), 'labels': [], 'sections': False, 'events': events,
    }, {'start_gap': start_gap, 'end_gap': end_gap, 'min_gap': min_gap, 'touched': touched, 'final': log.ticks[-1]}


# ---------------------------------------------------------------------------------------------- step 5
CORNER_START = (0.5, -1.0)         # where the bench's corner begins in the square100 field (heading 0, in the south corridor)
CORNER_SECONDS = 3.0               # of the bench's 4 s that the replay shows after the corner starts
SLOW_SERVO = dict(servo_delay=0.15, servo_tau=0.10)


def run_bench_corner(feed, damping, seconds=4.0, dt=0.02):
    """EXACTLY course/tests/helpers.py run_arc(arc_steer ...) with the slow servo: same car (seed 0), same warm-up on the
    straight to cruising speed, gyroscope bias calibrated away, rate = full-lock yaw rate, first corner to the left.

    The poses are translated so that the corner starts at CORNER_START; the warm-up before it is part of the replay.
    Returns (log, overshoot rad, t_close s, time the corner starts).
    """
    car = MiniCar(realistic=True, **SLOW_SERVO)
    car._bias = 0.0
    warm = Log()
    warm.pose(car)
    for _ in range(100):                                   # reach cruising speed on the straight
        car.step(1.28, 0.0, dt)
        warm.pose(car)
    start_time = car.time
    shift_x, shift_y = CORNER_START[0] - car.x, CORNER_START[1] - car.y
    car.x = car.y = car.yaw = 0.0                          # the bench restarts the pose here (time too; we keep ours)
    goal, ref = math.pi / 2, 0.0
    rate = real_speed(1.28) * math.tan(STEER_LIMIT) / WHEELBASE
    log = Log()
    for row in warm.ticks:
        log.ticks.append((row[0], row[1] + shift_x, row[2] + shift_y) + tuple(row[3:]))
    overshoot, t_close = 0.0, None
    t0 = car.time
    for _ in range(int(seconds / dt)):
        steer, ref = arc_steer(goal, ref, car.yaw, car.gyro(), rate, dt, feed=feed, damping=damping)
        car.step(1.28, steer, dt)
        overshoot = max(overshoot, car.yaw - goal)
        if t_close is None and abs(goal - car.yaw) <= 0.12:
            t_close = car.time - t0
        log.ticks.append((car.time, car.x + CORNER_START[0], car.y + CORNER_START[1], car.yaw, car.steer, car.speed))
    return log, overshoot, t_close, start_time


def step5():
    main, over_main, close_main, t_turn = run_bench_corner(True, 0.1)
    ghost, over_ghost, close_ghost, _ = run_bench_corner(False, 0.0)
    # the same numbers the tests (course/tests/test_step_5.py, helpers.run_arc) get
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'tests'))
    try:
        from functools import partial
        from helpers import run_arc
        ref_main = run_arc(arc_steer, **SLOW_SERVO)['overshoot']
        ref_ghost = run_arc(partial(arc_steer, feed=False, damping=0.0), **SLOW_SERVO)['overshoot']
        assert abs(ref_main - over_main) < 1e-12 and abs(ref_ghost - over_ghost) < 1e-12, (ref_main, over_main, ref_ghost, over_ghost)
        bench = 'igual que helpers.run_arc'
    except ImportError:
        bench = 'no comprobado contra helpers.run_arc'
    duration = round(t_turn + CORNER_SECONDS, 2)
    peak_ghost = max((row for row in ghost.ticks if row[0] >= t_turn), key=lambda row: row[3])[0]
    deg_main, deg_ghost = math.degrees(over_main), math.degrees(over_ghost)
    events = [
        {'t': 0.0, 'text': 'Parte del reposo y alcanza la velocidad de crucero (0,6 m/s) en el tramo recto'},
        {'t': round(t_turn, 2), 'text': 'Empieza la esquina: el objetivo gira 90° a la izquierda'},
        {'t': round(t_turn + 0.3, 2), 'text': 'Experimento con servo lento (retardo 0,15 s, lag 0,10 s), no el coche real'},
        {'t': round(peak_ghost, 2), 'text': 'Solo rampa: se pasa %s° del rumbo' % es(deg_ghost, 1)},
        {'t': round(t_turn + close_main, 2), 'text': 'Con anticipación y amortiguamiento: se pasa solo %s°' % es(deg_main, 1)},
    ]
    return {
        'version': 1, 'step': 5, 'title': 'Paso 5: la esquina',
        'caption': 'Experimento con servo lento: la esquina solo con rampa se pasa %s° del rumbo de 90° y con anticipación y amortiguamiento, %s°. %s'
                   % (es(deg_ghost, 1), es(deg_main, 1), HONEST),
        'field': 'square100', 'direction': 'ccw', 'duration': duration,
        'tracks': [
            {'id': 'main', 'label': 'Con anticipación y amortiguamiento', 'color': MAIN_COLOR, 'frames': main.frames(duration)},
            {'id': 'ghost', 'label': 'Solo rampa', 'color': GHOST_COLOR, 'frames': ghost.frames(duration)},
        ],
        'trail': True, 'scans': [], 'scene': [],
        'labels': [{'t': 0.0, 'state': 'FOLLOW', 'corners': 0}, {'t': round(t_turn, 2), 'state': 'TURN', 'corners': 0},
                   {'t': round(t_turn + close_main, 2), 'state': 'FOLLOW', 'corners': 1}],
        'sections': False, 'events': sorted(events, key=lambda e: e['t']),
    }, {'overshoot_main_deg': deg_main, 'overshoot_ramp_only_deg': deg_ghost, 't_close_main': close_main,
        't_close_ramp_only': close_ghost, 'bench': bench,
        'touch_main': any(Field(WALLS).touches(*row[1:4]) for row in main.ticks),
        'touch_ghost': any(Field(WALLS).touches(*row[1:4]) for row in ghost.ticks),
        'start_x': main.ticks[0][1]}


# ---------------------------------------------------------------------------------------------- validation
def validate(path, data):
    text = json.dumps(data, separators=(',', ':'), ensure_ascii=False, allow_nan=False)
    with open(path, 'w') as handle:
        handle.write(text)
    loaded = json.load(open(path))
    frames = loaded['tracks'][0]['frames']
    n = len(frames)
    assert all(len(t['frames']) == n for t in loaded['tracks']), 'tracks with different frame counts'
    for track in loaded['tracks']:
        ts = [f[0] for f in track['frames']]
        assert all(abs(b - a - FRAME_DT) < 1e-6 for a, b in zip(ts, ts[1:])), 'dt is not fixed'
        assert abs(ts[-1] - loaded['duration']) < 1e-6, 'duration %s != last frame %s' % (loaded['duration'], ts[-1])
    for key in ('scans', 'scene', 'events', 'labels'):
        times = [item['t'] for item in loaded[key]]
        assert times == sorted(times), key + ' not sorted'
        assert all(0 <= t <= loaded['duration'] + 1e-6 for t in times), key + ' outside the duration'
    for scan in loaded['scans']:
        assert len(scan['ranges']) == SCAN_RAYS
    size = os.path.getsize(path)
    assert size < 400_000, 'file too big: %d' % size
    return size, n, len(loaded['scans'])


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else OUT_DEFAULT
    os.makedirs(out, exist_ok=True)
    rows, notes = [], {}
    for builder in (step1, step2, step3, step4, step5):
        data, info = builder()
        path = os.path.join(out, 'step-%d.json' % data['step'])
        size, frames, scans = validate(path, data)
        rows.append((os.path.basename(path), size, data['duration'], frames, scans, len(data['scene']), len(data['events'])))
        notes[data['step']] = info
    print('%-12s %9s %9s %7s %6s %7s %7s' % ('file', 'bytes', 'duration', 'frames', 'scans', 'scenes', 'events'))
    for row in rows:
        print('%-12s %9d %9.2f %7d %6d %7d %7d' % row)
    for step, info in notes.items():
        print('step', step, info)


if __name__ == '__main__':
    main()
