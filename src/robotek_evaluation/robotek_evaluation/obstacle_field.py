"""The Obstacle Challenge field by the rules: traffic signs, the parking lot and the Gazebo world.

Simulation only (the strategy never imports it). ``robotek_course.obstacle_field`` re-exports it for the course.

* ``regulation_signs(seed, parking)``: where the signs go, with the procedure of the rules (section 8): two coins
  choose the section with a single sign and one coin its colour; 35 of the 36 cards (figure 8c) are drawn without
  replacement for the following straights, clockwise. Each straight has seats at its two ends and in its middle
  (along -0.5, 0, +0.5 m), 400 mm (inner row) or 600 mm (outer row) from the inner wall. With the parking lot,
  the signs of the start section move to the inner row (step 4 of the procedure). The start section is always the
  bottom straight here (the field is symmetric, so this loses nothing).
* ``PARKING_LOT``: the lot against the outer wall of the start section, 20 cm deep and 1.5 car lengths long, one
  limit next to the section line (x = 0.48).
* ``build_world(directory, seed, parking, direction, spawn_progress)``: writes ``world.sdf`` (the Open field + the
  signs, 50 x 50 x 100 mm, red RGB(238, 39, 55) and green RGB(68, 214, 44), with a contact sensor each and a pose
  publisher + the magenta lot limits, 200 x 20 x 100 mm) and ``manifest.json`` (direction, signs, lot, start).

The 36 cards are approximated from figure 8c of the rules: 11 single-sign cards (6 seats x 2 colours, minus the
middle outer-row seat of the removed card 9 or 10) and 16 two-sign cards (both ends, any row and colour).
"""
import json
import random
import re
import xml.etree.ElementTree as ET
from pathlib import Path

from robotek_common.vehicle import LENGTH

# Straights by the CCW tangent and the outward normal; cards are read with the inner wall on top.
SECTIONS = {'bottom': ((1, 0), (0, -1)), 'right': ((0, 1), (1, 0)), 'top': ((-1, 0), (0, 1)),
            'left': ((0, -1), (-1, 0))}
CLOCKWISE = ['bottom', 'left', 'top', 'right']
SINGLE_CARDS = [(pos, row, color) for row in (.4, .6) for pos in (-.5, 0., .5) for color in ('green', 'red')]
DOUBLE_CARDS = [((-.5, lrow, lcol), (.5, rrow, rcol)) for lrow in (.4, .6) for rrow in (.4, .6)
                for lcol in ('green', 'red') for rcol in ('green', 'red')]
PARKING_LOT = dict(x0=.48 - 1.5 * LENGTH, x1=.48, y0=-1.5, y1=-1.3)
SIGN_SIZE = (.05, .05, .10)
COLORS = {'red': '0.933 0.153 0.216 1', 'green': '0.267 0.839 0.173 1'}
SPAWN_PROGRESS = -.25        # m along the start straight (x = -0.25 CCW, +0.25 CW), y = -1.0


def regulation_signs(seed, parking=False):
    """The signs of one layout: [dict(name, color, x, y, side, seat=dict(along_ccw_m, from_inner_wall_m), size_m)]."""
    rng = random.Random(seed)
    single = rng.choice(CLOCKWISE)
    color = rng.choice(['green', 'red'])
    plan = {single: [(0., .6, color)]}      # the removed card 9/10 is the middle, outer-row seat
    deck = [[c] for c in SINGLE_CARDS if not (c[0] == 0. and c[1] == .6)] + [list(c) for c in DOUBLE_CARDS]
    rng.shuffle(deck)
    start = CLOCKWISE.index(single)
    for k in range(1, 4):
        plan[CLOCKWISE[(start + k) % 4]] = deck.pop()
    signs = []
    for side, cards in plan.items():
        (tx, ty), (nx, ny) = SECTIONS[side]
        for along, row, col in cards:
            if parking and side == 'bottom':
                row = .4
            lateral = .5 + row      # distance from the field centre along the outward normal
            signs.append(dict(color=col, x=round(tx * along + nx * lateral, 3), y=round(ty * along + ny * lateral, 3),
                              side=side, seat=dict(along_ccw_m=along, from_inner_wall_m=row),
                              size_m=list(SIGN_SIZE)))
    counts = {'red': 0, 'green': 0}
    for sign in signs:
        counts[sign['color']] += 1
        sign['name'] = 'sign_%s_%d' % (sign['color'], counts[sign['color']])
    return signs


def worlds_dir():
    """robotek_gazebo's worlds: the installed share folder, or the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('robotek_gazebo')) / 'worlds'
        if path.is_dir():
            return path
    except Exception:
        pass
    return Path(__file__).resolve().parents[2] / 'robotek_gazebo' / 'worlds'


def _contact_sensor(link, collision, topic):
    sensor = ET.SubElement(link, 'sensor', name='contact', type='contact')
    contact = ET.SubElement(sensor, 'contact')
    ET.SubElement(contact, 'collision').text = collision
    ET.SubElement(contact, 'topic').text = topic       # gz-sim reads the topic here, not under <sensor>
    ET.SubElement(sensor, 'always_on').text = '1'
    ET.SubElement(sensor, 'update_rate').text = '100'


def _box(link, name, size, rgba):
    for kind in ('collision', 'visual'):
        element = ET.SubElement(link, kind, name='%s_%s' % (name, kind))
        ET.SubElement(ET.SubElement(element, 'geometry'), 'box')
        ET.SubElement(element.find('geometry/box'), 'size').text = ' '.join(str(v) for v in size)
        if kind == 'visual':
            material = ET.SubElement(element, 'material')
            for key in ('ambient', 'diffuse'):
                ET.SubElement(material, key).text = rgba


def _signs_model(world, signs):
    """Movable signs (120 g, they can be pushed or knocked over), each with a contact sensor, and a pose publisher."""
    model = ET.SubElement(world, 'model', name='traffic_signs')
    ET.SubElement(model, 'pose').text = '0 0 0 0 0 0'
    for sign in signs:
        link = ET.SubElement(model, 'link', name=sign['name'])
        ET.SubElement(link, 'pose').text = '%s %s %s 0 0 0' % (sign['x'], sign['y'], SIGN_SIZE[2] / 2)
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = '0.12'
        inertia = ET.SubElement(inertial, 'inertia')
        for key, value in dict(ixx='0.000125', ixy='0', ixz='0', iyy='0.000125', iyz='0', izz='0.00005').items():
            ET.SubElement(inertia, key).text = value
        _box(link, 'sign', SIGN_SIZE, COLORS[sign['color']])
        _contact_sensor(link, 'sign_collision', '/obstacle/contacts')
    plugin = ET.SubElement(model, 'plugin', filename='gz-sim-pose-publisher-system',
                           name='gz::sim::systems::PosePublisher')
    for key, value in dict(publish_model_pose='true', publish_link_pose='true', use_pose_vector_msg='true',
                           update_frequency='20').items():
        ET.SubElement(plugin, key).text = value
    return model


def _lot_model(world):
    """The two magenta limits of the lot, static, with contact sensors (touching them ends the parking)."""
    model = ET.SubElement(world, 'model', name='parking_lot')
    ET.SubElement(model, 'static').text = 'true'
    lot = PARKING_LOT
    for name, x in (('parking_limit_a', lot['x0'] - .01), ('parking_limit_b', lot['x1'] + .01)):
        link = ET.SubElement(model, 'link', name=name)
        ET.SubElement(link, 'pose').text = '%s %s 0.05 0 0 0' % (x, (lot['y0'] + lot['y1']) / 2)
        _box(link, name, (.02, .20, .10), '1 0 1 1')
        _contact_sensor(link, '%s_collision' % name, '/obstacle/contacts')
    return model


def build_world(directory, seed, parking=True, direction='ccw', spawn_progress=SPAWN_PROGRESS):
    """Write ``world.sdf`` and ``manifest.json`` of one Obstacle round into ``directory``; returns the manifest."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    text = re.sub(r'<!--.*?-->', '', (worlds_dir() / 'robotek_open_field.sdf').read_text(), flags=re.S)
    tree = ET.ElementTree(ET.fromstring(text))
    world = tree.getroot().find('world')
    world.set('name', 'robotek_obstacle_round')
    signs = regulation_signs(seed, parking)
    _signs_model(world, signs)
    if parking:
        _lot_model(world)
    tree.write(directory / 'world.sdf', encoding='unicode', xml_declaration=True)
    sign = 1 if direction == 'ccw' else -1
    manifest = dict(challenge='obstacle', direction=direction, seed=seed, pillars=signs,
                    parking_lot=PARKING_LOT if parking else None, spawn_progress=spawn_progress,
                    spawn=dict(x=spawn_progress * sign, y=-1.0, yaw=0.0 if sign > 0 else 3.141592653589793),
                    honest=True, real_car=True)
    (directory / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest

