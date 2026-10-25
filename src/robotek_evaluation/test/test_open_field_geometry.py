"""Verify legal wall draws and changed-island clearance assessment."""

import importlib.util
import itertools
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from robotek_evaluation.open_field_geometry import field_geometry, inner_clearance


ROOT = Path(__file__).resolve().parents[3]
TEMPLATE = ROOT / 'src/robotek_gazebo/worlds/robotek_open_field.sdf'
SPEC = importlib.util.spec_from_file_location(
    'make_open_field', ROOT / 'src/robotek_evaluation/tools/make_open_field.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.mark.parametrize('draw', list(itertools.product((0.6, 1.0), repeat=4)))
def test_coin_draws_keep_exterior_and_visual_collision_agreement(tmp_path, draw):
    output = tmp_path / 'field.sdf'
    expected = dict(zip(('west', 'east', 'south', 'north'), draw))
    MODULE.generate(TEMPLATE, output, **expected)
    geometry = field_geometry(output)
    assert geometry['corridors'] == pytest.approx(expected)
    assert geometry['outer'] == pytest.approx(
        {'xmin': -1.5, 'xmax': 1.5, 'ymin': -1.5, 'ymax': 1.5})
    original = ET.parse(TEMPLATE).find(".//model[@name='game_field']/link")
    actual = ET.parse(output).find(".//model[@name='game_field']/link")
    for side in ('west', 'east', 'south', 'north'):
        visual = actual.find(f"visual[@name='wall_int_{side}']")
        collision = actual.find(f"collision[@name='wall_int_{side}_col']")
        for field in ('pose', 'geometry/box/size'):
            assert visual.findtext(field) == collision.findtext(field)
        for kind in ('visual', 'collision'):
            suffix = '_col' if kind == 'collision' else ''
            query = f"{kind}[@name='wall_ext_{side}{suffix}']"
            for field in ('pose', 'geometry/box/size'):
                assert original.find(query).findtext(field) == actual.find(query).findtext(field)
    assert (original.find("visual[@name='start_zone']/pose").text
            == actual.find("visual[@name='start_zone']/pose").text)


def test_clearance_changes_when_island_expands_or_moves(tmp_path):
    output = tmp_path / 'field.sdf'
    MODULE.generate(TEMPLATE, output, west=0.6, east=0.6, south=0.6, north=0.6)
    narrow = field_geometry(output)
    assert narrow['island']['width'] == pytest.approx(1.8)
    assert inner_clearance(0.75, 0, narrow['inner']) < 0
    assert inner_clearance(0.75, 0, field_geometry(TEMPLATE)['inner']) > 0
    MODULE.generate(TEMPLATE, output, west=1.0, east=0.6, south=0.6, north=1.0)
    shifted = field_geometry(output)
    assert shifted['island'] == pytest.approx(
        {'width': 1.4, 'height': 1.4, 'center_x': 0.2, 'center_y': -0.2})


@pytest.mark.parametrize('width', [0.5, 0.7, 0.9, 1.1])
def test_international_tolerance_extremes(tmp_path, width):
    output = tmp_path / 'field.sdf'
    MODULE.generate(TEMPLATE, output, west=width, east=width, south=width, north=width)
    assert field_geometry(output)['corridors']['south'] == pytest.approx(width)


def test_width_between_allowed_tolerance_bands_is_rejected(tmp_path):
    with pytest.raises(ValueError):
        MODULE.generate(TEMPLATE, tmp_path / 'field.sdf', east=0.8)
