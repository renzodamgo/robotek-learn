"""Sweeps of the Open lab: change one thing, run the 12 standard rounds, tabulate.

    source tools/macos_env.sh; source install/setup.zsh       # RoboStack, as for the rest of the course
    python3 course/maintainers/tuning/lab_sweep.py GROUP [GROUP ...] [--seeds 0,1,2] [--jobs 12]
    python3 course/maintainers/tuning/lab_sweep.py --list

Every row is one configuration of the whole suite (6 fields x 2 directions x the seeds). The columns:

* ok: rounds that are valid (30 points, no contact, stop observed, margin >= 1 cm, steering <= 25.5 deg);
* pts: mean points; t: mean time from Start of the 30-point rounds; sq100: the time on square100 (the Gazebo field);
* clr: the smallest clearance of the car outline to a wall in the whole suite, cm;
* wall: rounds that ended by touching a wall (and in which state); stopped short: no wall, but fewer than 30 points
  (the car stopped outside the start section, or never finished).

The raw per-round results go to ``raw/GROUP.json``; the tables to ``raw/GROUP.md``.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys

import numpy as np

from robotek_evaluation import open_lab as lab

HERE = Path(__file__).resolve().parent
STRATEGY = 'my_open.strategy:Strategy'

# The lab models the car as Gazebo shows it: the servo's dead time and lag are the yaml values plus what the
# Gazebo plugin adds (measured: 0.04 s and 0.01 s). A row "delay x2" means the yaml steer_delay 0.12 s.
DELAY0, TAU0, RATE0 = 0.06, 0.05, 1.2
PLUGIN_DELAY, PLUGIN_TAU = 0.04, 0.01


def servo(delay=DELAY0, tau=TAU0, rate=RATE0):
    """The actuator override for a servo with these model values (the Gazebo plugin's own lag is added)."""
    return dict(steer_delay=delay + PLUGIN_DELAY, steer_tau=tau + PLUGIN_TAU, steer_rate=rate)


def scaled_table(scale):
    return dict(speed_table_real=[x * scale for x in lab.actuator_parameters()['speed_table_real']])


def exp(label, sets=(), act=None, latency=0.0, fields=None, directions=None, note=''):
    return dict(label=label, sets=list(sets), act=act, latency=latency, fields=fields, directions=directions, note=note)


def S(name, value, pkg='my_open.*'):
    return f'{pkg}.{name}={value}'


GROUPS = {}


def group(name, title):
    def wrap(fn):
        GROUPS[name] = (title, fn)
        return fn
    return wrap


@group('baseline', 'Baseline')
def _():
    return [exp('baseline')]


@group('servo_delay', 'Servo dead time (yaml steer_delay; x1 = 0.06 s)')
def _():
    return [exp(f'delay x{k:g} = {DELAY0 * k:.2f} s', act=servo(delay=DELAY0 * k)) for k in (0.5, 1, 1.5, 2, 3, 5)]


@group('servo_lag', 'Servo lag (yaml steer_tau; x1 = 0.05 s)')
def _():
    return [exp(f'lag x{k:g} = {TAU0 * k:.2f} s', act=servo(tau=TAU0 * k)) for k in (0.5, 1, 2, 4, 8)]


@group('servo_rate', 'Servo speed limit (steer_rate; x1 = 1.2 rad/s = 69 deg/s)')
def _():
    return [exp(f'rate {r:g} rad/s', act=servo(rate=r)) for r in (4.4, 2.4, 1.2, 0.9, 0.6, 0.4, 0.3, 0.2)]


@group('servo_fix', 'Servo: slow servo (delay x3) with different fixes')
def _():
    slow = servo(delay=DELAY0 * 3)
    rows = [exp('delay x3, no change', act=slow)]
    rows += [exp(f'delay x3, DAMPING {d:g}', sets=[S('DAMPING', d)], act=slow) for d in (0.0, 0.05, 0.2, 0.3)]
    rows += [exp(f'delay x3, EXIT_ANGLE {e:g}', sets=[S('EXIT_ANGLE', e)], act=slow) for e in (0.1, 0.4, 0.6)]
    rows += [exp(f'delay x3, TURN_FRONT {t:g}', sets=[S('TURN_FRONT', t)], act=slow) for t in (1.25, 1.3, 1.4)]
    rows += [exp(f'delay x3, cruise {c:g}', sets=[S('CRUISE_SPEED', c, 'my_open.mission')], act=slow)
             for c in (0.9, 0.7)]
    return rows


@group('steer_limit', 'Maximum steering angle: what the car does (strategy still assumes 0.307 rad)')
def _():
    return [exp(f'car reaches {a:g} rad ({np.degrees(a):.1f} deg)', act=dict(steer_limit=a))
            for a in (0.20, 0.25, 0.28, 0.307, 0.35, 0.40, 0.4363)]


@group('steer_assumed', 'Maximum steering angle: strategy and car agree (both change)')
def _():
    rows = []
    for a in (0.20, 0.25, 0.28, 0.307, 0.35, 0.40, 0.4363):
        rows.append(exp(f'both {a:g} rad ({np.degrees(a):.1f} deg)', act=dict(steer_limit=a),
                        sets=[S('STEER_LIMIT', a), S('STEER_LIMIT', a, 'robotek_course.minisim')]))
    return rows


@group('steer_assumed_only', 'Maximum steering angle: strategy assumes it but the car stays at 0.307')
def _():
    return [exp(f'strategy assumes {a:g} rad ({np.degrees(a):.1f} deg)',
                sets=[S('STEER_LIMIT', a), S('STEER_LIMIT', a, 'robotek_course.minisim')])
            for a in (0.20, 0.25, 0.35, 0.40, 0.4363)]


@group('cruise', 'Commanded cruise speed (the car reaches the table value)')
def _():
    return [exp(f'cruise {c:g} m/s', sets=[S('CRUISE_SPEED', c, 'my_open.mission')])
            for c in (0.4, 0.6, 0.8, 1.0, 1.1, 1.28)]


@group('last_corner', 'Speed of the last corner and of the finish')
def _():
    rows = [exp(f'last corner {c:g}', sets=[S('LAST_CORNER_SPEED', c, 'my_open.mission')]) for c in (0.3, 0.6, 0.9, 1.28)]
    rows += [exp(f'finish {c:g}', sets=[S('FINISH_SPEED', c, 'my_open.mission')]) for c in (0.08, 0.3, 0.6)]
    return rows


@group('motor', 'Motor: the speed the car really reaches (table scale; strategy keeps its table)')
def _():
    return [exp(f'real speed x{k:g}', act=scaled_table(k)) for k in (0.5, 0.7, 0.85, 1.0, 1.15, 1.3, 1.5)]


@group('motor_aware', 'Motor: same, but the strategy table is also updated')
def _():
    base = [(0.0, 0.0), (0.15, 0.18), (0.30, 0.27), (0.40, 0.30), (0.60, 0.42), (1.28, 0.60)]
    rows = []
    for k in (0.7, 0.85, 1.15, 1.3):
        table = [[c, round(r * k, 4)] for c, r in base]
        rows.append(exp(f'real speed x{k:g}, table updated', act=scaled_table(k),
                        sets=[f'robotek_course.minisim.SPEED_TABLE={json.dumps(table)}']))
    return rows


@group('motor_slow', 'Motor: slower response (drive_tau, drive_delay)')
def _():
    rows = [exp(f'drive_tau {t:g} s', act=dict(drive_tau=t)) for t in (0.05, 0.15, 0.3, 0.6)]
    rows += [exp(f'drive_delay {d:g} s', act=dict(drive_delay=d)) for d in (0.0, 0.06, 0.15, 0.3)]
    rows += [exp(f'accel_max {a:g} m/s2', act=dict(accel_max=a)) for a in (0.3, 0.6, 2.0)]
    return rows


@group('lane', 'Lane keeping: GAP, GAIN, LIMIT')
def _():
    rows = [exp(f'GAP {g:g} m', sets=[S('GAP', g, 'my_open.lane'), S('GAP', g, 'my_open.mission')])
            for g in (0.15, 0.20, 0.25, 0.30, 0.35, 0.40)]
    rows += [exp(f'GAIN {g:g}', sets=[S('GAIN', g)]) for g in (0.0, 0.1, 0.6, 1.0, 2.0)]
    rows += [exp(f'LIMIT {g:g} rad', sets=[S('LIMIT', g)]) for g in (0.05, 0.2, 0.6)]
    return rows


@group('kp', 'Heading gain KP (steering per radian of heading error)')
def _():
    return [exp(f'KP {k:g}', sets=[S('KP', k)]) for k in (0.2, 0.4, 0.8, 1.2, 2.0, 3.0, 5.0)]


@group('damping', 'Corner: DAMPING and EXIT_ANGLE')
def _():
    rows = [exp(f'DAMPING {d:g}', sets=[S('DAMPING', d)]) for d in (0.0, 0.05, 0.1, 0.2, 0.4, 0.8)]
    rows += [exp(f'EXIT_ANGLE {e:g} rad', sets=[S('EXIT_ANGLE', e)]) for e in (0.02, 0.1, 0.25, 0.4, 0.6, 0.9)]
    return rows


@group('fade', 'Corner when the car turns FASTER than the strategy believes (car wheelbase 25 % shorter): DAMPING, EXIT_ANGLE')
def _():
    car = dict(wheelbase=lab.actuator_parameters()['wheelbase'] * 0.75)
    rows = [exp('car turns 33 % faster, defaults', act=car)]
    rows += [exp(f'faster, DAMPING {d:g}', sets=[S('DAMPING', d)], act=car) for d in (0.0, 0.3)]
    rows += [exp(f'faster, EXIT_ANGLE {e:g}', sets=[S('EXIT_ANGLE', e)], act=car) for e in (0.02, 0.6, 0.9)]
    rows += [exp('faster, DAMPING 0 + EXIT_ANGLE 0.02', sets=[S('DAMPING', 0.0), S('EXIT_ANGLE', 0.02)], act=car)]
    return rows


@group('gate', 'When to turn: TURN_FRONT, LOOKAHEAD, ARM_DISTANCE')
def _():
    rows = [exp(f'TURN_FRONT {t:g} m', sets=[S('TURN_FRONT', t)]) for t in (0.9, 1.0, 1.1, 1.19, 1.3, 1.4, 1.6)]
    rows += [exp(f'LOOKAHEAD {t:g} m', sets=[S('LOOKAHEAD', t)]) for t in (0.1, 0.2, 0.35, 0.5, 0.8)]
    rows += [exp(f'ARM_DISTANCE {t:g} m', sets=[S('ARM_DISTANCE', t)]) for t in (0.0, 0.1, 0.25, 0.5, 0.8, 1.2)]
    return rows


@group('sensors', 'Sensors: LiDAR dropout and range noise, gyroscope bias and noise')
def _():
    L = 'robotek_evaluation.open_lab'
    rows = [exp(f'LiDAR dropout {d:.0%}', sets=[S('SCAN_DROPOUT', d, L)]) for d in (0.0, 0.043, 0.08, 0.12, 0.15, 0.25, 0.4)]
    rows += [exp(f'LiDAR range noise {n * 1000:g} mm', sets=[S('SCAN_NOISE', n, L)]) for n in (0.0, 0.008, 0.02, 0.04, 0.08)]
    rows += [exp(f'gyro bias {b:g} rad/s', sets=[S('GYRO_BIAS', b, L)]) for b in (0.0, 0.006, 0.02, 0.05, 0.1)]
    rows += [exp(f'gyro noise {n:g} rad/s', sets=[S('GYRO_NOISE', n, L)]) for n in (0.0, 0.0019, 0.01, 0.03, 0.1)]
    return rows


@group('gyro_drift', 'Gyroscope: drift after the calibration and scale error')
def _():
    L = 'robotek_evaluation.open_lab'
    rows = [exp(f'gyro drift {d:g} rad/s per s', sets=[S('GYRO_DRIFT', d, L)]) for d in (0.00002, 0.00005, 0.0001, 0.0002, 0.0005)]
    rows += [exp(f'gyro scale {s:g}', sets=[S('GYRO_SCALE', s, L)]) for s in (0.9, 0.95, 0.97, 0.98, 0.99, 1.01, 1.02, 1.03, 1.05)]
    return rows


@group('calibration', 'Calibration time (CALIBRATE_SECONDS) with different gyro biases')
def _():
    L = 'robotek_evaluation.open_lab'
    rows = []
    for c in (0.05, 0.1, 0.3, 1.0, 2.0):
        rows.append(exp(f'calibrate {c:g} s', sets=[S('CALIBRATE_SECONDS', c, 'my_open.mission')]))
    for c in (0.05, 0.3):
        rows.append(exp(f'calibrate {c:g} s, gyro noise 0.01', sets=[S('CALIBRATE_SECONDS', c, 'my_open.mission'),
                                                                     S('GYRO_NOISE', 0.01, L)]))
    return rows


@group('camera', 'Camera: slower to see the direction')
def _():
    L = 'robotek_evaluation.open_lab'
    rows = [exp(f'camera ready after {d:g} s', sets=[S('CAMERA_DELAY', d, L)]) for d in (0.3, 1.0, 3.0, 8.0, 14.0, 16.0)]
    rows += [exp(f'camera {1 / p:.0f} fps', sets=[S('CAMERA_PERIOD', p, L)]) for p in (1 / 6, 1 / 3, 1 / 1)]
    return rows


@group('latency', 'Control latency (Pi/ROS between the strategy and the actuators)')
def _():
    return [exp(f'latency {x * 1000:g} ms', latency=x) for x in (0.0, 0.02, 0.05, 0.1, 0.2)]


@group('wheelbase', 'Wheelbase: a longer or shorter car, code NOT updated vs updated')
def _():
    from robotek_common import vehicle
    rows = []
    base_act = lab.actuator_parameters()['wheelbase']
    for dl in (-0.02, 0.0, 0.03, 0.06, 0.12):
        w = vehicle.WHEELBASE + dl
        k = w / vehicle.WHEELBASE
        car = dict(wheelbase=base_act * k)
        # geometry of the judge's outline and of the rear axle (the car turns about it)
        geo = [S('WHEELBASE', w, 'robotek_common.vehicle'), S('REAR_AXLE_X', -w / 2, 'robotek_common.vehicle'),
               S('OUTLINE_FRONT', vehicle.OUTLINE_FRONT + dl / 2, 'robotek_common.vehicle'),
               S('OUTLINE_REAR', vehicle.OUTLINE_REAR + dl / 2, 'robotek_common.vehicle')]
        # NOT updated: the strategy and drive.py keep assuming the old wheelbase (the lab's Actuators convert with it)
        rows.append(exp(f'wheelbase {w * 100:.1f} cm, strategy not updated', sets=[geo[1], geo[2], geo[3]], act=car))
        if dl:
            rows.append(exp(f'wheelbase {w * 100:.1f} cm, WHEELBASE updated', act=car,
                            sets=geo + [S('WHEELBASE', w), S('WHEELBASE', w, 'robotek_course.minisim')]))
    return rows


MISSION = 'my_open.mission'
FAST = [S('LAST_CORNER_SPEED', 1.1, MISSION), S('FINISH_SPEED', 0.6, MISSION)]       # found by search_speed.py
SAFER = [S('LAST_CORNER_SPEED', 0.9, MISSION), S('FINISH_SPEED', 0.45, MISSION)]
STEER25 = [S('STEER_LIMIT', 0.4363), S('STEER_LIMIT', 0.4363, 'robotek_course.minisim')]


@group('candidates', 'Faster candidates, 3 seeds (use --seeds 0,1,2)')
def _():
    return [exp('baseline'),
            exp('last corner 1.1 + finish 0.6 (search result)', sets=FAST),
            exp('last corner 0.9 + finish 0.45 (safer)', sets=SAFER),
            exp('search result + GAIN 0.6', sets=FAST + [S('GAIN', 0.6)]),
            exp('search result + steering 25 deg (car and strategy)', sets=FAST + STEER25, act=dict(steer_limit=0.4363)),
            exp('last corner 1.28 (as cruise) + finish 0.6', sets=[S('LAST_CORNER_SPEED', 1.28, MISSION), S('FINISH_SPEED', 0.6, MISSION)])]


@group('candidates_robust', 'Baseline vs search result when something else is off')
def _():
    rows = []
    cases = [('motor x0.85', scaled_table(0.85)), ('motor x1.15', scaled_table(1.15)),
             ('servo delay x2', servo(delay=DELAY0 * 2)), ('servo rate 0.6', servo(rate=0.6))]
    for name, act in cases:
        rows.append(exp(f'baseline, {name}', act=act))
        rows.append(exp(f'search result, {name}', sets=FAST, act=act))
        rows.append(exp(f'safer, {name}', sets=SAFER, act=act))
    L = 'robotek_evaluation.open_lab'
    rows.append(exp('baseline, LiDAR dropout 12%', sets=[S('SCAN_DROPOUT', 0.12, L)]))
    rows.append(exp('search result, LiDAR dropout 12%', sets=FAST + [S('SCAN_DROPOUT', 0.12, L)]))
    return rows


@group('cruise_high', 'Commanded speed above the motor limit (1.28 = 100 % motor)')
def _():
    return [exp(f'cruise {c:g}', sets=[S('CRUISE_SPEED', c, MISSION)]) for c in (1.28, 1.6, 2.0)]


# ------------------------------------------------------------------------------------------------ run + report
def summarize(label, results):
    ok = [r for r in results if r['valid']]
    full = [r for r in results if r['points'] >= 30 and r['time']]
    sq = [r['time'] for r in results if r['field'] == 'square100' and r['points'] >= 30 and r['direction'] == 'ccw' and r['time']]
    clr = [r['min_clearance'] for r in results if r['min_clearance'] is not None]
    walls = Counter(r['end'] for r in results if r['contact_time'] is not None)
    stuck = Counter(r['end'] for r in results if r['contact_time'] is None and r['points'] < 30)
    why = Counter(f for r in results for f in r['failed'])
    return dict(label=label, ok=len(ok), n=len(results), pts=float(np.mean([r['points'] for r in results])),
                t=float(np.mean([r['time'] for r in full])) if full else None, sq100=sq[0] if sq else None,
                clr=min(clr) * 100 if clr else None, walls=dict(walls), stuck=dict(stuck), why=dict(why))


def run_group(name, seeds, jobs):
    title, build = GROUPS[name]
    rows, raw = [], []
    for e in build():
        results = lab.run_suite_parallel(STRATEGY, {}, e['sets'], seeds, e['fields'], e['directions'],
                                         actuator=e['act'], latency=e['latency'], jobs=jobs)
        row = summarize(e['label'], results)
        rows.append(row)
        raw.append(dict(experiment=e, summary=row, rounds=results))
        print(format_row(row), flush=True)
    (HERE / 'raw' / f'{name}.json').write_text(json.dumps(
        dict(group=name, title=title, seeds=seeds, experiments=raw), indent=1, default=float))
    table = f'### {title}\n\nSeeds {",".join(map(str, seeds))}, {rows[0]["n"]} rounds per row.\n\n' + markdown(rows) + '\n'
    (HERE / 'raw' / f'{name}.md').write_text(table)
    return table


def fmt(x, spec):
    return '-' if x is None else format(x, spec)


def format_row(r):
    return '%-46s ok %2d/%-2d pts %5.1f t %6s sq100 %6s clr %5s  wall %s short %s' % (
        r['label'], r['ok'], r['n'], r['pts'], fmt(r['t'], '.2f'), fmt(r['sq100'], '.2f'), fmt(r['clr'], '.1f'),
        r['walls'] or '-', r['stuck'] or '-')


def markdown(rows):
    lines = ['| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |', '|---|---:|---:|---:|---:|---:|---|']
    for r in rows:
        fail = '; '.join([f'wall in {k} x{v}' for k, v in r['walls'].items()] +
                         [f'stopped short in {k} x{v}' for k, v in r['stuck'].items()]) or ''
        lines.append(f"| {r['label']} | {r['ok']}/{r['n']} | {r['pts']:.1f} | {fmt(r['t'], '.2f')} | "
                     f"{fmt(r['sq100'], '.2f')} | {fmt(r['clr'], '.1f')} | {fail} |")
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('groups', nargs='*')
    parser.add_argument('--seeds', default='0')
    parser.add_argument('--jobs', type=int, default=12)
    parser.add_argument('--list', action='store_true')
    args = parser.parse_args()
    if args.list or not args.groups:
        for k, (title, _) in GROUPS.items():
            print(f'{k:20s} {title}')
        return 0
    seeds = [int(s) for s in args.seeds.split(',')]
    for g in (list(GROUPS) if args.groups == ['all'] else args.groups):
        print('\n##', GROUPS[g][0], flush=True)
        run_group(g, seeds, args.jobs)
    return 0


if __name__ == '__main__':
    sys.exit(main())
