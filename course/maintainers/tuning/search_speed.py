"""Coordinate search for a faster, still safe configuration of my_open in the Open lab.

    python3 course/maintainers/tuning/search_speed.py [--max-evals 25] [--seeds 0,1] [--jobs 12]

Objective: every round valid (30 points, no contact) with a smallest wall clearance of at least ``--min-clearance`` cm
(default 5, the judge's "comfortable"), and then the smallest mean time over the 12 standard rounds x the seeds.
Starts from the baseline, tries each candidate value of one constant at a time and keeps the best; it passes over
the constants until a full pass changes nothing. The state (``raw/search_state.json``) lets you resume it.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from robotek_evaluation import open_lab as lab

HERE = Path(__file__).resolve().parent
STATE = HERE / 'raw' / 'search_state.json'
STRATEGY = 'my_open.strategy:Strategy'

# name -> (module pattern that holds it, candidate values)
SPACE = {
    'LAST_CORNER_SPEED': ('my_open.mission', [0.6, 0.8, 0.9, 1.0, 1.1]),
    'FINISH_SPEED': ('my_open.mission', [0.15, 0.3, 0.45, 0.6, 0.8]),
    'BRAKE_GAIN': ('my_open.mission', [1.0, 2.0, 3.0, 4.0]),
    'BRAKE_MARGIN': ('my_open.mission', [0.05, 0.15, 0.25, 0.35]),
    'FINISH_LEAD': ('my_open.mission', [0.04, 0.08, 0.15]),
    'CALIBRATE_SECONDS': ('my_open.mission', [0.1, 0.2, 0.3]),
    'KP': ('my_open.*', [0.6, 0.8, 1.0, 1.2]),
    'TURN_FRONT': ('my_open.*', [1.1, 1.19, 1.3]),
    'ARM_DISTANCE': ('my_open.*', [0.1, 0.25, 0.4]),
    'GAP': ('my_open.*', [0.15, 0.2, 0.25]),
    'GAIN': ('my_open.*', [0.3, 0.6, 1.0]),
}
BASE = {'LAST_CORNER_SPEED': 0.6, 'FINISH_SPEED': 0.15, 'BRAKE_GAIN': 2.0, 'BRAKE_MARGIN': 0.15, 'FINISH_LEAD': 0.08,
        'CALIBRATE_SECONDS': 0.3, 'KP': 0.8, 'TURN_FRONT': 1.19, 'ARM_DISTANCE': 0.25, 'GAP': 0.25, 'GAIN': 0.3}


def sets_of(config):
    return [f'{SPACE[k][0]}.{k}={v}' for k, v in config.items() if v != BASE[k]]


def evaluate(config, seeds, jobs):
    results = lab.run_suite_parallel(STRATEGY, {}, sets_of(config), seeds, jobs=jobs)
    ok = all(r['valid'] for r in results)
    clearance = min((r['min_clearance'] for r in results if r['min_clearance'] is not None), default=0.0) * 100
    times = [r['time'] for r in results if r['time']]
    return dict(valid=ok, clr=clearance, time=float(np.mean(times)) if times else None,
                n_valid=sum(r['valid'] for r in results), n=len(results))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--max-evals', type=int, default=25)
    parser.add_argument('--seeds', default='0,1')
    parser.add_argument('--jobs', type=int, default=12)
    parser.add_argument('--min-clearance', type=float, default=5.0, help='cm')
    parser.add_argument('--reset', action='store_true')
    args = parser.parse_args()
    seeds = [int(s) for s in args.seeds.split(',')]
    state = json.loads(STATE.read_text()) if STATE.exists() and not args.reset else dict(
        best=dict(BASE), best_eval=None, log=[], cursor=0, changed_in_pass=False, passes=0, done=False)
    if state['done']:
        print('done:', state['best'], state['best_eval'])
        return 0
    names = list(SPACE)
    evals = 0
    if state['best_eval'] is None:
        state['best_eval'] = evaluate(state['best'], seeds, args.jobs)
        state['log'].append(dict(config=dict(state['best']), **state['best_eval']))
        print('baseline', state['best_eval'], flush=True)
    while evals < args.max_evals and not state['done']:
        name = names[state['cursor']]
        for value in SPACE[name][1]:
            if value == state['best'][name]:
                continue
            trial = {**state['best'], name: value}
            e = evaluate(trial, seeds, args.jobs)
            evals += 1
            state['log'].append(dict(config={k: v for k, v in trial.items()}, **e))
            good = e['valid'] and e['clr'] >= args.min_clearance and e['time'] is not None
            better = good and e['time'] < state['best_eval']['time'] - 0.02
            print(f'{name}={value}: {e["n_valid"]}/{e["n"]} valid, clr {e["clr"]:.1f} cm, '
                  f't {e["time"] if e["time"] is None else round(e["time"], 2)}{"  <- better" if better else ""}', flush=True)
            if better:
                state['best'], state['best_eval'], state['changed_in_pass'] = trial, e, True
        state['cursor'] += 1
        if state['cursor'] == len(names):
            state['cursor'], state['passes'] = 0, state['passes'] + 1
            state['done'] = not state['changed_in_pass']
            state['changed_in_pass'] = False
        STATE.write_text(json.dumps(state, indent=1))
    STATE.write_text(json.dumps(state, indent=1))
    print('best so far:', state['best'], state['best_eval'], 'passes', state['passes'], 'cursor', state['cursor'])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
