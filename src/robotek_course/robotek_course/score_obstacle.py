"""Score a recorded Obstacle round with the Obstacle judge (``robotek_evaluation.obstacle_judge``).

    ros2 run robotek_course score_obstacle                 # the latest Obstacle round in ~/robotek_runs
    ros2 run robotek_course score_obstacle <round folder>

The strategy writes nothing: the referee keeps the true pose of the car (``audit_pose.csv``), the Start
(``events.jsonl``), the field (``manifest.json``, ``world.sdf``), the pose of the signs (``pillars-gt.jsonl``) and
the contacts (``contacts.jsonl``). The final rest of the car is marked as DONE (as ``score`` does for the Open) and
the judge counts what the rules award:

  1.1  one point per section left in the direction of the round (24 at most);
  1.2  one point per lap (3);
  1.3  three points for stopping inside the start section after the three laps (30 s observed);
  1.4-1.7  the signs: 10 after three laps if none moved, 8 if one did;
  1.8  with the parking lot: 15 parked fully inside and parallel, 7 partly or not parallel.

Faults end the round: a sign passed on the wrong side (9.24.5) or moved out of its circle (9.24.6). A round is
*valid* with the maximum (40, or 55 with the lot), no faults, no contacts, 30 s of stop and the steering within
25 degrees. All of it is a simulation estimate, not an official score.
"""
import argparse
import json
from pathlib import Path

from robotek_evaluation.obstacle_judge import judge_obstacle, read_evidence

from .score import RUNS, mark_done


def score_directory(directory, write=True):
    """Judge one Obstacle round folder. Returns the report and writes rule_judge.json and round.json."""
    directory = Path(directory)
    manifest = json.loads((directory / 'manifest.json').read_text())
    rows, events = read_evidence(directory)
    events = mark_done(rows, events)
    for row in rows:
        row.pop('state', None)          # the judge derives DONE from the done event
    report = judge_obstacle(directory, manifest, rows, events)
    if write:
        (directory / 'rule_judge.json').write_text(json.dumps(
            dict(rule_judge=report['rule_judge'], validation=report['validation']), indent=2, default=float) + '\n')
        (directory / 'round.json').write_text(json.dumps(report, indent=2, default=float) + '\n')
    return report


def format_report(directory, report):
    judge, validation = report['rule_judge'], report['validation']
    b = judge['breakdown']
    parts = (f'secciones {b["1.1_sections"]} + vueltas {b["1.2_laps"]} + parada {b["1.3_stop_in_finish"]}'
             f' + señales {b["1.4-1.7_traffic_signs"]}')
    if b.get('1.8_parking') is not None:
        parts += f' + aparcamiento {b["1.8_parking"]}'
    lines = [f'Ronda: {directory}', f'Puntos: {judge["points"]} de {judge["max_points"]}   ({parts})']
    if judge.get('time_from_start_sim_seconds') is not None:
        laps = ' / '.join(f'{lap["sim_seconds"]:.1f}' for lap in judge['lap_splits'])
        lines.append(f'Tiempo desde Start: {judge["time_from_start_sim_seconds"]:.2f} s simulados'
                     + (f'   (vueltas: {laps} s)' if laps else ''))
    for fault in judge['faults']:
        lines.append(f'Falta: {fault["fault"]} (regla {fault["rule"]}) a los {fault["sim_time"]:.1f} s'
                     + (f', señal {fault["pillar"]}' if fault.get('pillar') else ''))
    park = report.get('parking')
    if park:
        lines.append(f'Aparcamiento: {park["verdict"]}')
    margin = validation.get('min_clearance_m')
    lines.append('Válida: ' + ('sí' if validation['passed'] else 'no (' + ', '.join(validation['failed']) + ')')
                 + (f'   margen mínimo {margin * 100:.1f} cm' if margin is not None else ''))
    lines.append('Son estimaciones de simulación, no puntuación oficial.')
    return '\n'.join(lines)


def latest_run(root=RUNS):
    runs = sorted(p for p in Path(root).expanduser().glob('*')
                  if (p / 'audit_pose.csv').exists() and (p / 'pillars-gt.jsonl').exists())
    if not runs:
        raise SystemExit(f'No hay rondas del Obstacle en {root}: corre antes  '
                         'ros2 launch robotek_course obstacle_round.launch.py')
    return runs[-1]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('directory', nargs='?', type=Path, help='round folder (default: the latest Obstacle round)')
    parser.add_argument('--json', action='store_true', help='print the whole report as JSON')
    args = parser.parse_args(argv)
    directory = args.directory or latest_run()
    report = score_directory(directory)
    print(json.dumps(report, indent=2, default=float) if args.json else format_report(directory, report))


if __name__ == '__main__':
    main()
