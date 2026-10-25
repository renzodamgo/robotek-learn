"""Step 9: score and measure (RESULTADOS.md and a round of the Obstacle lab)."""
import re
from pathlib import Path

import pytest

PACKAGE = Path(__file__).resolve().parents[3] / 'src' / 'my_obstacle'


def test_the_results_are_written_down():
    text = (PACKAGE / 'RESULTADOS.md').read_text()
    assert re.search(r'^\|.*[Pp]untos', text, re.M)                     # a table with points
    assert 'Gazebo' in text and 'laboratorio' in text.lower()
    assert re.search(r'\d+ ?/ ?\d+', text)                               # rounds counted, valid of total


def test_a_round_of_the_lab_scores_the_three_laps(tmp_path):
    lab = pytest.importorskip('robotek_evaluation.obstacle_lab')
    directory = lab.make_field(str(tmp_path), 4, 'ccw', False)
    result = lab.run_round(directory, lab.make_strategy('my_obstacle.strategy:Strategy', False), seed=0)
    assert result['points'] == 40 and result['valid']
    assert result['time'] < 120.0
