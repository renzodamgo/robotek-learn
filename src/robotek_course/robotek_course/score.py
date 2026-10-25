"""Score a recorded round with the same rule judge the Gazebo runs of the team use.

    ros2 run robotek_course score                 # the latest round in ~/robotek_runs
    ros2 run robotek_course score <round folder>

The strategy writes nothing: the round recorder keeps the true pose of the car (``audit_pose.csv``), the Start
(``events.jsonl``) and the field (``manifest.json``, ``world.sdf``). Here the moment the car stopped for good is
found in that pose and marked as DONE, and ``judge_open`` counts what the rules award:

  1.1  one point per section the car drives out of, in the direction of the round (24 at most);
  1.2  one point per lap (3);
  1.3  three points for stopping with the whole car inside the start section (still for 15 s).

A round is *valid* when it also has the maximum, no faults, 30 s of stop observed, no contact with a wall, margin
to the walls and a steering angle within 25 degrees (``standard_validation``). All of it is a simulation estimate,
not an official score.
"""
import argparse
import csv
import json
import math
from pathlib import Path

from robotek_evaluation.open_field_geometry import inner_clearance
from robotek_evaluation.rule_judge import judge_open, load_contacts, standard_validation

STILL_M = 0.001                       # the final rest: every pose within 1 mm of the last one
RUNS = Path.home() / 'robotek_runs'


def read_rows(directory):
    """The true poses of the round, as dictionaries of text, in time order."""
    with (directory / 'audit_pose.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    required = ('sim_time', 'wall_time', 'x_gt', 'y_gt', 'yaw_gt', 'steer_actual', 'steer_right_actual')
    rows = [r for r in rows if all(r.get(k) not in (None, '', 'nan') and math.isfinite(float(r[k])) for k in required)]
    rows.sort(key=lambda r: float(r['sim_time']))
    return rows


def read_events(directory):
    path = directory / 'events.jsonl'
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()] if path.exists() else []


def mark_done(rows, events):
    """Mark as DONE the final rest of the car and add the ``done`` event (what the strategy used to write).

    The rest starts at the earliest pose from which every later pose is within ``STILL_M`` of the last one.
    The judge only looks at it after the 24 section exits, so a car that stops early is simply not judged as done.
    """
    if not rows:
        return events
    x_end, y_end = float(rows[-1]['x_gt']), float(rows[-1]['y_gt'])
    first = len(rows) - 1
    while first > 0 and math.hypot(float(rows[first - 1]['x_gt']) - x_end, float(rows[first - 1]['y_gt']) - y_end) <= STILL_M:
        first -= 1
    for row in rows:
        row['state'] = 'DONE' if float(row['sim_time']) >= float(rows[first]['sim_time']) else ''
    events = [e for e in events if e['event'] != 'done']
    events.append(dict(event='done', source='score', sim_time=float(rows[first]['sim_time']),
                       wall_time=float(rows[first]['wall_time'])))
    return events


def clearances(rows, manifest, t0, t1):
    """Closest the car's body came to the outer wall and to the island (sampled), in metres."""
    geometry = manifest.get('field_geometry') or {}
    inner, outer = geometry.get('inner'), geometry.get('outer')
    poses = [(float(r['x_gt']), float(r['y_gt']), float(r['yaw_gt'])) for r in rows if t0 <= float(r['sim_time']) <= t1]
    if not (poses and inner and outer):
        return dict(outer_wall=None, inner_wall=None)
    outer_min = min(min(min(x - outer['xmin'], outer['xmax'] - x) - (0.089 * abs(math.cos(yaw)) + 0.05 * abs(math.sin(yaw))),
                        min(y - outer['ymin'], outer['ymax'] - y) - (0.089 * abs(math.sin(yaw)) + 0.05 * abs(math.cos(yaw))))
                    for x, y, yaw in poses)
    return dict(outer_wall=outer_min, inner_wall=min(inner_clearance(x, y, inner) for x, y, _ in poses))


def score_directory(directory, write=True):
    """Judge one round folder. Returns {'rule_judge': ..., 'validation': ...} and writes rule_judge.json."""
    directory = Path(directory)
    manifest = json.loads((directory / 'manifest.json').read_text())
    rows = read_rows(directory)
    events = mark_done(rows, read_events(directory))
    judge = judge_open(directory, manifest, rows, events)
    start = next((e for e in events if e['event'] == 'start'), None)
    end = (judge.get('round_end') or {}).get('sim_time', math.inf)
    contacts = load_contacts(directory)
    touching = None if contacts is None else [c for c in contacts if c['sim_time'] <= end]
    steer = [abs(float(r[k])) for r in rows for k in ('steer_actual', 'steer_right_actual')]
    validation = standard_validation(
        judge, clearances(rows, manifest, float(start['sim_time']) if start else 0.0, end),
        contacts is not None, touching, judge.get('stop_hold_sim_seconds'),
        math.degrees(max(steer)) if steer else None)
    result = dict(rule_judge=judge, validation=validation)
    if write:
        (directory / 'rule_judge.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def format_report(directory, result):
    judge, validation = result['rule_judge'], result['validation']
    b = judge['breakdown']
    lines = [f'Ronda: {directory}',
             f'Puntos: {judge["points"]} de {judge["max_points"]}   '
             f'(secciones {b["1.1_sections"]} + vueltas {b["1.2_laps"]} + parada {b["1.3_stop_in_finish"]})']
    if judge.get('time_from_start_sim_seconds') is not None:
        laps = ' / '.join(f'{lap["sim_seconds"]:.1f}' for lap in judge['lap_splits'])
        lines.append(f'Tiempo desde Start: {judge["time_from_start_sim_seconds"]:.2f} s simulados'
                     + (f'   (vueltas: {laps} s)' if laps else ''))
    if (judge.get('round_end') or {}).get('reason') not in (None, 'stopped_after_three_laps'):
        lines.append(f'La ronda terminó por: {judge["round_end"]["reason"]}')
    for fault in judge['faults']:
        lines.append(f'Falta: {fault["fault"]} (regla {fault["rule"]}) a los {fault["sim_time"]:.1f} s')
    for note in judge['notes']:
        if note.startswith(('Stop not counted', 'Three laps but', 'No start event')):
            lines.append('Aviso: ' + note)
    margin = validation.get('min_clearance_m')
    lines.append('Válida: ' + ('sí' if validation['passed'] else 'no (' + ', '.join(validation['failed']) + ')')
                 + (f'   margen mínimo {margin * 100:.1f} cm' if margin is not None else ''))
    lines.append('Son estimaciones de simulación, no puntuación oficial.')
    return '\n'.join(lines)


def latest_run(root=RUNS):
    runs = sorted(p for p in Path(root).expanduser().glob('*') if (p / 'audit_pose.csv').exists())
    if not runs:
        raise SystemExit(f'No hay rondas en {root}: corre antes  ros2 launch robotek_course round.launch.py')
    return runs[-1]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('directory', nargs='?', type=Path, help='round folder (default: the latest in ~/robotek_runs)')
    parser.add_argument('--json', action='store_true', help='print the whole judge result as JSON')
    args = parser.parse_args(argv)
    directory = args.directory or latest_run()
    result = score_directory(directory)
    print(json.dumps(result, indent=2) if args.json else format_report(directory, result))


if __name__ == '__main__':
    main()
