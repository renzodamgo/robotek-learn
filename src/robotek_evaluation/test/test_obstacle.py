"""Obstacle infrastructure: the field by the rules, the world, the lab and the judge (no ROS, no Gazebo)."""
import json
import math
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from robotek_evaluation import obstacle_lab as lab
from robotek_evaluation.obstacle_field import PARKING_LOT, build_world, regulation_signs
from robotek_evaluation.obstacle_judge import judge_obstacle, judge_parking


def test_layouts_are_reproducible_and_follow_the_cards():
    assert regulation_signs(7) == regulation_signs(7)
    for seed in range(40):
        signs = regulation_signs(seed)
        by_side = {}
        for sign in signs:
            by_side.setdefault(sign['side'], []).append(sign)
            assert sign['seat']['along_ccw_m'] in (-0.5, 0.0, 0.5)
            assert sign['seat']['from_inner_wall_m'] in (0.4, 0.6)
        assert set(by_side) == {'bottom', 'right', 'top', 'left'}
        for side_signs in by_side.values():
            if len(side_signs) == 2:          # two-sign cards always use both ends of the straight
                assert sorted(s['seat']['along_ccw_m'] for s in side_signs) == [-0.5, 0.5]
            else:
                assert len(side_signs) == 1


def test_the_lot_moves_the_start_section_signs_inward():
    for seed in range(40):
        moved = [s for s in regulation_signs(seed, parking=True) if s['side'] == 'bottom']
        assert all(s['seat']['from_inner_wall_m'] == 0.4 and s['y'] == pytest.approx(-0.9) for s in moved)
        others = [(s['x'], s['y']) for s in regulation_signs(seed) if s['side'] != 'bottom']
        assert others == [(s['x'], s['y']) for s in regulation_signs(seed, True) if s['side'] != 'bottom']


def test_the_world_has_every_sign_with_a_contact_sensor_and_the_lot(tmp_path):
    manifest = build_world(tmp_path, 3, parking=True, direction='cw')
    root = ET.parse(tmp_path / 'world.sdf').getroot()
    links = root.findall(".//model[@name='traffic_signs']/link")
    assert len(links) == len(manifest['pillars'])
    assert all(link.find("sensor[@type='contact']") is not None for link in links)
    assert root.find(".//model[@name='traffic_signs']/plugin[@name='gz::sim::systems::PosePublisher']") is not None
    assert len(root.findall(".//model[@name='parking_lot']/link")) == 2
    assert manifest['spawn']['x'] == pytest.approx(0.25) and manifest['spawn']['yaw'] == pytest.approx(math.pi)
    assert json.loads((tmp_path / 'manifest.json').read_text())['parking_lot'] == PARKING_LOT


def test_the_lab_world_sees_the_signs_and_the_lot(tmp_path):
    build_world(tmp_path, 0, parking=True)
    world = lab.World(tmp_path)
    assert 'parking_limit' in world.kinds
    sign = world.pillars[0]
    # A ray from 0.5 m beside a sign hits its near face (25 mm before the centre).
    hit = world.raycast(sign['x'] - 0.5, sign['y'], np.array([0.0]))[0]
    assert hit <= 0.5 - 0.025 + 1e-6


def test_the_camera_sees_only_inside_its_field_of_view(tmp_path):
    build_world(tmp_path, 0, parking=False)
    world = lab.World(tmp_path)
    world.pillars = [dict(x=1.0, y=0.0, color='red'), dict(x=0.0, y=1.0, color='green'), dict(x=3.0, y=0.0, color='red')]
    seen = lab.camera_observations(world, 0.0, 0.0, 0.0, np.random.default_rng(0))
    assert [o['color'] for o in seen] == ['red']        # ahead: yes; 90 deg left: no; 3 m: too far
    assert seen[0]['x'] == pytest.approx(1.0, abs=0.05)


class StandStill:
    """A strategy that never moves (the base API, without the base class)."""
    label, done = 'WAIT', False

    def on_scan(self, scan):
        pass

    def on_imu(self, now, gyro_z):
        pass

    def on_signs(self, observations, stamp):
        self.seen = observations

    def control(self, now, dt):
        return 0.0, 0.0


def test_a_car_that_stands_still_scores_nothing(tmp_path):
    build_world(tmp_path, 1, parking=False)
    result = lab.run_round(tmp_path, StandStill, tmax=3.0)
    assert result['points'] == 0 and not result['valid'] and result['contact'] is None


def _row(t, x, y, yaw=0.0):
    return dict(sim_time=t, wall_time=t, x_gt=x, y_gt=y, yaw_gt=yaw, steer_actual=0.0, steer_right_actual=0.0)


def test_parking_judge_fully_inside_and_parallel_is_15(tmp_path):
    (tmp_path / 'contacts.jsonl').write_text('')
    lot = PARKING_LOT
    x, y = (lot['x0'] + lot['x1']) / 2, (lot['y0'] + lot['y1']) / 2
    rows = [_row(t, x, y) for t in np.arange(0.0, 5.0, 0.1)]
    events = [dict(event='start', sim_time=0.0, wall_time=0.0), dict(event='done', sim_time=1.0, wall_time=1.0)]
    report = judge_parking(tmp_path, dict(parking_lot=lot, direction='ccw'), rows, events)
    assert report['verdict'] == 'parked_fully_parallel' and report['points_estimated'] == 15
    touched = dict(sim_time=2.0, kind='parking_limit', pillar=None, other='parking_lot::parking_limit_a',
                   robot='robotek::x', episode=0, end_sim_time=2.0, messages=1)
    (tmp_path / 'contacts.jsonl').write_text(json.dumps(touched) + '\n')
    assert judge_parking(tmp_path, dict(parking_lot=lot, direction='ccw'), rows, events)['points_estimated'] == 0


def test_round_judge_counts_no_laps_for_a_short_drive(tmp_path):
    manifest = build_world(tmp_path, 2, parking=False)
    (tmp_path / 'contacts.jsonl').write_text('')
    rows = [_row(t, -0.25 + 0.3 * t, -1.0) for t in np.arange(0.0, 2.0, 0.1)]
    report = judge_obstacle(tmp_path, manifest, rows, [dict(event='start', sim_time=0.0, wall_time=0.0)])
    assert report['laps_completed'] == 0 and report['rule_judge']['max_points'] == 40


def test_scan_clusters_find_a_sign_and_skip_walls():
    from robotek_course.obstacle_vision import scan_clusters
    angles = np.linspace(-math.radians(110), math.radians(110), 306)
    ranges = np.full(306, math.inf)
    ranges[np.abs(angles) < math.radians(1.5)] = 0.975          # a 5 cm sign face 0.975 m ahead
    ranges[angles > math.radians(60)] = 0.5                       # a wall on the left
    scan = SimpleNamespace(ranges=list(ranges), angle_min=angles[0], angle_increment=angles[1] - angles[0],
                           range_min=0.12, range_max=12.0)
    clusters = scan_clusters(scan)
    assert len(clusters) == 1
    assert clusters[0]['x'] == pytest.approx(0.975 + 0.025 + 0.128, abs=0.01)
