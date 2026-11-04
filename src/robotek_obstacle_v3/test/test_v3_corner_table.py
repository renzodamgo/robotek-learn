"""The precomputed v3 corner table matches the planner (WRO-Strategies' tools/build_corner_table.py built it).

The numbers are compared to 1e-12: the table was computed on another machine and the last bit of a float can
differ (2e-16 on the Mac, also in WRO-Strategies). A real change of geometry (robotek_common's vehicle.py instead of
car.py, for example) moves it by 1e-4 m and fails.
"""
import random

import pytest

from robotek_obstacle_v3 import corner
from robotek_obstacle_v3.corner_table import TABLE


def test_table_covers_the_mission_and_matches_the_planner():
    assert len(TABLE) > 500
    family = [args for args, value in TABLE.items() if value is None or len(value) != 2]
    for args in random.Random(0).sample(sorted(family, key=repr), 12):
        expected = corner._choose(args)
        if expected is None:
            assert TABLE[args] is None, args
            continue
        assert TABLE[args] == pytest.approx(tuple(float(v) for v in expected), abs=1e-12), args


def test_searched_lines_beat_the_family_line():
    searched = [args for args, value in TABLE.items() if value is not None and len(value) == 2]
    for args in random.Random(0).sample(sorted(searched, key=repr), min(4, len(searched))):
        gap, (kind, knots) = TABLE[args]
        assert kind == 'k' and len(knots) > 10
        assert gap >= corner._choose(args)[0] + 0.005, args
