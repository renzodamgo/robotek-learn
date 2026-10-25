#!/usr/bin/env python3
"""Rebuild the `solutions-obstacle` branch: one commit and one tag (obstacle-step-0 ... 9) per step of the guide.

For maintainers, not for readers. It starts from the finished package in src/my_obstacle (the reference solution)
and rebuilds it step by step, like build_solutions.py does for the Open:

* obstacle-step-0 is what `ros2 pkg create ... my_obstacle --node-name drive` prints (``--base``, or, without it,
  the my_open of tag step-0 with its names changed);
* step N adds the modules of step N (MODULES) and the `drive.py` of that step (course/snapshots/obstacle/
  drive_stepN.py for 2 <= N <= 6, the final one from 7 on);
* step 9 adds RESULTADOS.md (the table of the measured rounds).

    python3 course/maintainers/build_obstacle_solutions.py [--base /path/to/generated/my_obstacle]
    python3 course/maintainers/build_obstacle_solutions.py --repo /tmp/clone      # a dry run in a clone

Needs a clean checkout of `main` and replaces the `solutions-obstacle` branch and its tags. src/my_obstacle is ignored
on main, so every step is committed with `git add -f`.
"""
import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TITLES = {
    0: 'Preparar el paquete: tal como lo crea ros2 pkg create',
    1: 'El campo y las reglas: el carril y el progreso',
    2: 'Ubicarse con los muros: el localizador',
    3: 'Ver las señales y recordarlas por asiento',
    4: 'La línea: un lado por señal',
    5: 'Las esquinas: recta, arco, recta',
    6: 'Seguir la línea: curvatura y realimentación',
    7: 'La misión: salir, mirar, tres vueltas',
    8: 'Aparcar en paralelo',
    9: 'Puntuar y medir: las rondas',
}
MODULES = {1: ['lane.py'], 2: ['localizer.py'], 3: ['signs.py'], 4: ['plan.py'], 5: ['corner.py'],
           6: ['follow.py'], 7: ['mission.py', 'strategy.py'], 8: ['park.py'], 9: []}
STEP0 = ['LICENSE', 'package.xml', 'setup.cfg', 'setup.py', 'resource/my_open', 'my_open/__init__.py',
         'my_open/drive.py', 'test/test_copyright.py', 'test/test_flake8.py', 'test/test_pep257.py']


def git(root, *args, check=True):
    return subprocess.run(['git', *args], cwd=root, check=check, capture_output=True, text=True).stdout.strip()


def step0_from_open(root, target):
    """The package `ros2 pkg create` prints for my_obstacle, from the my_open of tag step-0."""
    for name in STEP0:
        text = git(root, 'show', f'step-0:src/my_open/{name}') + '\n'
        text = text.replace('my_open', 'my_obstacle').replace('Mi estrategia del reto Open',
                                                              'Mi estrategia del reto Obstacle')
        if name.startswith('resource/'):
            text = ''
        path = target / name.replace('my_open', 'my_obstacle')
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)


def build(step, package, base, final, snapshots):
    shutil.rmtree(package, ignore_errors=True)
    shutil.copytree(base, package)
    for n in range(1, step + 1):
        for name in MODULES[n]:
            shutil.copy(final / 'my_obstacle' / name, package / 'my_obstacle' / name)
    if 2 <= step <= 6:
        shutil.copy(snapshots / f'drive_step{step}.py', package / 'my_obstacle' / 'drive.py')
    elif step >= 7:
        shutil.copy(final / 'my_obstacle' / 'drive.py', package / 'my_obstacle' / 'drive.py')
    if step == 9:
        shutil.copy(final / 'RESULTADOS.md', package / 'RESULTADOS.md')
    for cache in package.rglob('__pycache__'):
        shutil.rmtree(cache)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--base', type=Path, help='the my_obstacle folder printed by ros2 pkg create')
    parser.add_argument('--branch', default='solutions-obstacle')
    parser.add_argument('--repo', type=Path, default=ROOT, help='the checkout to build in (default: this one)')
    parser.add_argument('--package', type=Path, default=ROOT / 'src' / 'my_obstacle',
                        help='the finished reference package (default: src/my_obstacle of this checkout)')
    parser.add_argument('--snapshots', type=Path, default=ROOT / 'course' / 'snapshots' / 'obstacle')
    args = parser.parse_args()
    root = args.repo.resolve()
    if git(root, 'status', '--porcelain', '--untracked-files=no'):
        sys.exit('The working tree must be clean (commit main first).')
    if git(root, 'rev-parse', '--abbrev-ref', 'HEAD') != 'main':
        sys.exit('Start from main.')
    work = Path(tempfile.mkdtemp())
    final = work / 'my_obstacle'
    shutil.copytree(args.package, final, ignore=shutil.ignore_patterns('__pycache__'))
    snapshots = work / 'snapshots'
    shutil.copytree(args.snapshots, snapshots)
    base = args.base
    if base is None:
        base = work / 'base'
        step0_from_open(root, base)
    package = root / 'src' / 'my_obstacle'
    git(root, 'branch', '-D', args.branch, check=False)
    git(root, 'checkout', '-b', args.branch)
    for step in range(10):
        build(step, package, base, final, snapshots)
        git(root, 'add', '-f', 'src/my_obstacle')
        git(root, 'commit', '-m', f'obstacle-step-{step}: {TITLES[step]}')
        git(root, 'tag', '-f', f'obstacle-step-{step}')
        print(f'obstacle-step-{step}', git(root, 'rev-parse', '--short', 'HEAD'))
    shutil.rmtree(root / 'course' / 'snapshots' / 'obstacle', ignore_errors=True)
    shutil.copytree(snapshots, root / 'course' / 'snapshots' / 'obstacle')
    git(root, 'add', '-f', 'course/snapshots/obstacle')
    git(root, 'commit', '-m', 'maintainers: the drive.py of every Obstacle step (built by course/maintainers)')
    print(f'Done. HEAD of {args.branch} is one commit after obstacle-step-9 (the snapshots); '
          'src/my_obstacle is the finished solution. Back to main with: git checkout main')


if __name__ == '__main__':
    main()
