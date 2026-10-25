"""The exam: your strategy in the Open lab, judged by the same rules as Gazebo (needs the ROS packages installed)."""
import pytest

pytest.importorskip('rclpy', reason='source ROS first (see step 0)')

from robotek_evaluation import open_lab as lab   # noqa: E402


def run(field, direction):
    return lab.run_round(lab.FIELDS / f'{field}.sdf', direction, lab.load_strategy('my_open.strategy:Strategy'))


@pytest.mark.parametrize('direction', ['ccw', 'cw'])
def test_square100_gives_30_points_and_is_valid(direction):
    result = run('square100', direction)
    assert result['points'] == 30 and result['valid'], result
    assert result['time'] < 34.0


def test_a_wide_field_in_both_directions():
    results = [run('square180', 'ccw'), run('rect100x180', 'cw')]
    assert all(r['points'] == 30 and r['valid'] for r in results), results
