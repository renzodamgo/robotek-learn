#!/usr/bin/env python3
"""Generate Open field wall placements from four drawn corridor widths."""

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


def generate(template, output, *, west=1.0, east=1.0, south=1.0, north=1.0):
    """Keep mat/exterior fixed and resize both visual and collision inner walls."""
    widths = [west, east, south, north]
    if any(not (0.5 <= value <= 0.7 or 0.9 <= value <= 1.1) for value in widths):
        raise ValueError('Corridors must fit the 0.6/1.0 m draws with ±0.1 m tolerance')
    tree = ET.parse(template)
    link = tree.find(".//model[@name='game_field']/link[@name='field']")
    xmin, xmax, ymin, ymax = -1.5 + west, 1.5 - east, -1.5 + south, 1.5 - north
    cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
    width, height = xmax - xmin, ymax - ymin
    for kind in ('visual', 'collision'):
        for side in ('west', 'east', 'south', 'north'):
            suffix = '_col' if kind == 'collision' else ''
            node = link.find(f"{kind}[@name='wall_int_{side}{suffix}']")
            size = [float(v) for v in node.findtext('geometry/box/size').split()]
            pose = [float(v) for v in node.findtext('pose').split()]
            if side in ('west', 'east'):
                pose[0] = xmin + size[0] / 2 if side == 'west' else xmax - size[0] / 2
                pose[1], size[1] = cy, height
            else:
                pose[0], size[0] = cx, width
                pose[1] = ymin + size[1] / 2 if side == 'south' else ymax - size[1] / 2
            node.find('pose').text = ' '.join(f'{v:.9g}' for v in pose)
            node.find('geometry/box/size').text = ' '.join(f'{v:.9g}' for v in size)
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space='  ')
    tree.write(output, encoding='utf-8', xml_declaration=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--template', type=Path, default=(
        Path(__file__).resolve().parents[3] / 'src/robotek_gazebo/worlds/robotek_open_field.sdf'))
    for side in ('west', 'east', 'south', 'north'):
        parser.add_argument('--' + side, type=float, default=1.0)
    args = parser.parse_args()
    corridors = {s: getattr(args, s) for s in ('west', 'east', 'south', 'north')}
    generate(args.template, args.output, **corridors)


if __name__ == '__main__':
    main()
