import math

import numpy as np

from robotek_course.minisim import Field
from my_open.perception import perceive


def scene_at(x, y, yaw=0.0, forward_yaw=0.0, sign=1, **options):
    return perceive(Field().scan(x, y, yaw, **options), forward_yaw=forward_yaw, sign=sign)


def test_the_distances_to_the_walls_at_each_side():
    scene = scene_at(0.0, -0.75, noise=False, dropouts=False)
    # The median of the beams within +-25 deg of the side: the oblique beams are longer, so it reads 2 % too far.
    assert math.isclose(scene.left, 0.25, abs_tol=0.02)      # the island, on the left of a counter-clockwise lap
    assert math.isclose(scene.right, 0.75, abs_tol=0.03)     # the outer wall


def test_the_wall_ahead():
    assert math.isclose(scene_at(0.0, -0.75, noise=False, dropouts=False).front_axis, 1.50 - 0.128, abs_tol=0.01)


def test_the_wall_ahead_is_measured_along_the_track_when_the_car_is_crooked():
    crooked = 0.3                                            # the car points 17 deg to the left of the track
    scene = scene_at(0.0, -0.75, yaw=crooked, forward_yaw=-crooked, noise=False, dropouts=False)
    assert math.isclose(scene.front_axis, 1.50 - 0.128 * math.cos(crooked), abs_tol=0.03)
    pointing = scene_at(0.0, -0.75, yaw=crooked, forward_yaw=0.0, noise=False, dropouts=False)
    assert abs(pointing.front_axis - scene.front_axis) > 0.03        # measuring along the car gives another number


def test_the_end_of_the_island():
    for x in (0.0, 0.1, 0.2):
        scene = scene_at(x, -0.75, noise=False, dropouts=False)
        assert math.isclose(scene.island_end, 0.5 - x, abs_tol=0.06), x


def test_no_island_end_when_it_is_not_in_view_or_not_beside_the_car():
    assert math.isinf(scene_at(-0.6, -0.75, noise=False, dropouts=False).island_end)       # still too far ahead
    past = scene_at(0.9, -0.75, noise=False, dropouts=False)                               # already beside the open side
    assert math.isinf(past.island_end) and past.left > 1.0


def test_a_clockwise_lap_is_the_mirror_image():
    ccw = scene_at(0.0, -0.75, noise=False, dropouts=False)
    cw = scene_at(0.0, 0.75, yaw=0.0, sign=-1, noise=False, dropouts=False)           # the north corridor, island on the right
    assert math.isclose(cw.right, ccw.left, abs_tol=0.01) and math.isclose(cw.left, ccw.right, abs_tol=0.01)
    assert math.isclose(cw.island_end, ccw.island_end, abs_tol=0.06)


def test_noise_and_lost_beams_do_not_move_the_numbers_much():
    clean = scene_at(0.1, -0.75, noise=False, dropouts=False)
    for seed in range(5):
        noisy = perceive(Field(seed=seed).scan(0.1, -0.75, 0.0))
        assert abs(noisy.left - clean.left) < 0.02 and abs(noisy.front_axis - clean.front_axis) < 0.02
        assert abs(noisy.island_end - clean.island_end) < 0.08


def test_an_empty_scan_gives_infinite_distances():
    class Empty:
        ranges = [math.inf] * 306
        angle_min, angle_increment, range_min, range_max = -1.91986, 2 * 1.91986 / 305, 0.12, 12.0
    scene = perceive(Empty())
    assert math.isinf(scene.left) and math.isinf(scene.right) and math.isinf(scene.front_axis) and math.isinf(scene.island_end)


def test_a_median_ignores_a_beam_that_goes_through_a_gap():
    scan = Field().scan(0.0, -0.75, 0.0, noise=False, dropouts=False)
    ranges = np.array(scan.ranges)
    side = int(round((math.pi / 2 - scan.angle_min) / scan.angle_increment))
    ranges[side] = 9.0                                       # one wild reading right at the side bearing
    scan.ranges = list(ranges)
    assert math.isclose(perceive(scan).left, 0.25, abs_tol=0.01)
