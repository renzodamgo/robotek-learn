#!/usr/bin/env python3
"""Check a step of the Obstacle course in Gazebo, with no window: one round and its score.

    python3 course/obstacle/verify_step.py N [--seed S] [--direction ccw|cw] [--parking|--no-parking]
                                             [--timeout 400] [--keep]

Run it where ROS and the course workspace are sourced (and the workspace with YOUR ``my_obstacle``, built for step
N, on top of it). It launches ``obstacle_round.launch.py strategy:=my_obstacle gui:=false`` (the simulator, your
node and the referee), waits for the round to end, reads the score the referee wrote (``round.json``) and prints
PASS or FAIL with the numbers. Exit code: 0 PASS, 1 FAIL, 2 the checker could not run (another Gazebo running, no
round folder...).

What each step must show (``EXPECT``):

    steps 1-6  your node runs for the whole round and the car is not in contact with anything; the round may end
               by time or by stalling (these steps do not drive the laps yet)
    step 7     three laps with the signs on their sides and the stop: 40 points, valid (no lot)
    step 8     with the parking lot: 55 points (three laps + parked fully inside and parallel), valid
    step 9     as step 8, on the layout and direction you choose (the measured rounds of RESULTADOS.md)

The default field is seed 4 counter-clockwise, a layout the reference solution passes in Gazebo.
"""
import argparse
import glob
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

EXPECT = {n: dict(points=None, valid=None, laps=None, no_contact=True) for n in range(1, 7)}
EXPECT.update({7: dict(points=40, valid=True, laps=3, no_contact=True, parking=False),
               8: dict(points=55, valid=True, laps=3, no_contact=True, parking=True),
               9: dict(points=55, valid=True, laps=3, no_contact=True, parking=True)})


def gazebo_running():
    out = subprocess.run(['pgrep', '-f', 'gz sim'], capture_output=True, text=True).stdout.split()
    return [p for p in out if int(p) != os.getpid()]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('step', type=int, choices=range(1, 10))
    parser.add_argument('--seed', type=int, default=4)
    parser.add_argument('--direction', choices=['ccw', 'cw'], default='ccw')
    parser.add_argument('--parking', dest='parking', action='store_true', default=None)
    parser.add_argument('--no-parking', dest='parking', action='store_false')
    parser.add_argument('--timeout', type=float, default=400.0, help='wall seconds for the whole round')
    parser.add_argument('--runs-dir', default=os.path.expanduser('~/robotek_runs'))
    parser.add_argument('--keep', action='store_true', help='keep Gazebo logs on screen')
    args = parser.parse_args(argv)
    expect = EXPECT[args.step]
    parking = args.parking if args.parking is not None else expect.get('parking', False)
    if gazebo_running():
        print('verify_step: ya hay un Gazebo corriendo; ciérralo antes (pkill -f "gz sim").')
        return 2
    name = 'verify-obstacle-step%d-%d-%s' % (args.step, args.seed, args.direction)
    before = set(glob.glob(os.path.join(args.runs_dir, '*-' + name)))
    command = ['ros2', 'launch', 'robotek_course', 'obstacle_round.launch.py', 'strategy:=my_obstacle', 'gui:=false',
               'seed:=%d' % args.seed, 'direction:=%s' % args.direction, 'parking:=%s' % str(parking).lower(),
               'name:=%s' % name, 'runs_dir:=%s' % args.runs_dir]
    print('$ ' + ' '.join(command), flush=True)
    proc = subprocess.Popen(command, stdout=None if args.keep else subprocess.DEVNULL,
                            stderr=subprocess.STDOUT, start_new_session=True)
    try:
        proc.wait(timeout=args.timeout)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGINT)
        time.sleep(5)
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGKILL)
        print('verify_step: la ronda no terminó en %.0f s (¿tu nodo se colgó?)' % args.timeout)
    runs = sorted(set(glob.glob(os.path.join(args.runs_dir, '*-' + name))) - before)
    if not runs or not (Path(runs[-1]) / 'round.json').exists():
        print('verify_step: no hay round.json; mira la salida con --keep')
        return 2
    report = json.loads((Path(runs[-1]) / 'round.json').read_text())
    judge, validation = report['rule_judge'], report['validation']
    contacts = [line for line in (Path(runs[-1]) / 'contacts.jsonl').read_text().splitlines() if line.strip()] \
        if (Path(runs[-1]) / 'contacts.jsonl').exists() else []
    checks = []
    if expect['points'] is not None:
        checks.append(('puntos', judge['points'], expect['points'], judge['points'] >= expect['points']))
    if expect['laps'] is not None:
        checks.append(('vueltas', report['laps_completed'], expect['laps'], report['laps_completed'] >= expect['laps']))
    if expect['valid'] is not None:
        checks.append(('válida', validation['passed'], True, validation['passed']))
    if expect['no_contact']:
        checks.append(('contactos', len(contacts), 0, not contacts))
    print('Ronda: %s' % runs[-1])
    for name_, value, wanted, ok in checks:
        print('  %-10s %-8s (se espera %s)  %s' % (name_, value, wanted, 'ok' if ok else 'NO'))
    passed = all(ok for *_, ok in checks)
    print('PASS' if passed else 'FAIL')
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
