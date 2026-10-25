"""Read axis-aligned Open Challenge wall geometry from the actual SDF."""

import math
import xml.etree.ElementTree as ET


def field_geometry(world_path):
    """Return wall faces, island dimensions and corridor widths in metres."""
    link = ET.parse(world_path).find(".//model[@name='game_field']/link[@name='field']")
    if link is None:
        raise ValueError('World has no game_field/field wall geometry')

    def wall(kind, side):
        node = link.find(f"collision[@name='wall_{kind}_{side}_col']")
        if node is None:
            raise ValueError(f'Missing {kind} {side} wall')
        pose = [float(v) for v in node.findtext('pose').split()]
        size = [float(v) for v in node.findtext('geometry/box/size').split()]
        if any(not math.isfinite(v) for v in pose + size):
            raise ValueError('Nonfinite wall geometry')
        if any(abs(v) > 1e-9 for v in pose[3:]) or min(size) <= 0:
            raise ValueError('Expected positive, axis-aligned wall boxes')
        return pose, size

    bounds = {}
    for kind in ('int', 'ext'):
        current = {}
        for side, axis, key, sign in (
                ('west', 0, 'xmin', -1), ('east', 0, 'xmax', 1),
                ('south', 1, 'ymin', -1), ('north', 1, 'ymax', 1)):
            pose, size = wall(kind, side)
            face = sign if kind == 'int' else -sign
            current[key] = pose[axis] + face * size[axis] / 2
        if current['xmin'] >= current['xmax'] or current['ymin'] >= current['ymax']:
            raise ValueError('Inverted field bounds')
        bounds['inner' if kind == 'int' else 'outer'] = current
    inner, outer = bounds['inner'], bounds['outer']
    corridors = {
        'west': inner['xmin'] - outer['xmin'], 'east': outer['xmax'] - inner['xmax'],
        'south': inner['ymin'] - outer['ymin'], 'north': outer['ymax'] - inner['ymax'],
    }
    if min(corridors.values()) <= 0:
        raise ValueError('Island must fit inside the exterior walls')
    bounds.update(corridors=corridors, island={
        'width': inner['xmax'] - inner['xmin'], 'height': inner['ymax'] - inner['ymin'],
        'center_x': (inner['xmin'] + inner['xmax']) / 2,
        'center_y': (inner['ymin'] + inner['ymax']) / 2,
    })
    return bounds


def inner_clearance(x, y, bounds, radius=0.102):
    """Conservative body-circle clearance to an axis-aligned island rectangle."""
    dx = max(bounds['xmin'] - x, 0.0, x - bounds['xmax'])
    dy = max(bounds['ymin'] - y, 0.0, y - bounds['ymax'])
    return math.hypot(dx, dy) - radius
