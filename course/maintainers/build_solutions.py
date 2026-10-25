#!/usr/bin/env python3
"""Rebuild the `solutions` branch: one commit and one tag (step-0 ... step-9) per step of the guide.

For maintainers, not for readers. It starts from the finished package in src/my_open (the reference solution) and
rebuilds it step by step:

* step-0 is exactly what `ros2 pkg create` prints (the command of the README);
* step N adds the modules of step N (MODULES) and the `drive.py` of that step (course/snapshots/drive_stepN.py for
  N <= 7, the final one from 8 on);
* step 9 adds RESULTADOS.md, the table of the measured rounds.

    python3 course/maintainers/build_solutions.py --base /path/to/generated/my_open [--branch solutions]

Needs a clean checkout of `main` and replaces the `solutions` branch. src/my_open is ignored on main, so every step
is committed with `git add -f`.
"""
import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / 'src' / 'my_open'
TITLES = {
    0: 'Instalar y preparar: el paquete tal como lo crea ros2 pkg create',
    1: 'El coche: el modelo de bicicleta y un círculo',
    2: 'El rumbo: el giroscopio y la ley de rumbo',
    3: 'Ver el campo: el LiDAR y la escena',
    4: 'Seguir la isla: la ley de carril',
    5: 'La esquina: rampa, anticipación y amortiguamiento',
    6: 'Cuándo girar: la puerta de giro',
    7: 'La misión: tres vueltas y parar',
    8: 'La dirección: votación de la cámara',
    9: 'Puntuar y medir: las 12 rondas',
}
MODULES = {1: ['model.py'], 2: ['heading.py'], 3: ['perception.py'], 4: ['lane.py'], 5: ['arc.py'],
           6: ['turn_gate.py'], 7: ['mission.py', 'strategy.py'], 8: ['direction.py'], 9: []}


def git(*args, check=True):
    return subprocess.run(['git', *args], cwd=ROOT, check=check, capture_output=True, text=True).stdout.strip()


def build(step, base, final, extra):
    shutil.rmtree(PACKAGE, ignore_errors=True)
    shutil.copytree(base, PACKAGE)
    for n in range(1, step + 1):
        for name in MODULES[n]:
            shutil.copy(final / 'my_open' / name, PACKAGE / 'my_open' / name)
    if 1 <= step <= 7:
        shutil.copy(ROOT / 'course' / 'snapshots' / f'drive_step{step}.py', PACKAGE / 'my_open' / 'drive.py')
    elif step >= 8:
        shutil.copy(final / 'my_open' / 'drive.py', PACKAGE / 'my_open' / 'drive.py')
    if step == 9:
        shutil.copy(extra, PACKAGE / 'RESULTADOS.md')
    for cache in PACKAGE.rglob('__pycache__'):
        shutil.rmtree(cache)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--base', type=Path, required=True, help='the my_open folder printed by ros2 pkg create')
    parser.add_argument('--results', type=Path, required=True, help='RESULTADOS.md for step 9')
    parser.add_argument('--branch', default='solutions')
    args = parser.parse_args()
    if git('status', '--porcelain'):
        sys.exit('The working tree must be clean (commit main first).')
    if git('rev-parse', '--abbrev-ref', 'HEAD') != 'main':
        sys.exit('Start from main.')
    final = Path(tempfile.mkdtemp()) / 'my_open'
    shutil.copytree(PACKAGE, final, ignore=shutil.ignore_patterns('__pycache__'))
    git('branch', '-D', args.branch, check=False)
    git('checkout', '-b', args.branch)
    for step in range(10):
        build(step, args.base, final, args.results)
        git('add', '-f', 'src/my_open')
        git('commit', '-m', f'step-{step}: {TITLES[step]}')
        git('tag', '-f', f'step-{step}')
        print(f'step-{step}', git('rev-parse', '--short', 'HEAD'))
    git('add', '-f', 'course/snapshots')
    git('commit', '-m', 'maintainers: the drive.py of every step (the script that builds this branch is course/maintainers)')
    print(f'Done. HEAD of {args.branch} is one commit after step-9 (the snapshots); src/my_open is the finished solution. Back to main with: git checkout main')


if __name__ == '__main__':
    main()
