"""Open v5, layer 1: the Scene from hand-built LiDAR scans of the standard square field."""
import dataclasses
import math

import numpy as np
import pytest

import square_scan
from robotek_open.v5.scene import Scene, scene_from_scan

INNER = 0.5          # island half width (square100)


def scan_at(x, y=-1.15, yaw=0.0, inner=INNER, noise=0.0, seed=1):
    ranges, angle_min, increment = square_scan.raycast(
        x, y, yaw, inner, np.random.default_rng(seed), noise=noise)
    return square_scan.make_scan(ranges, angle_min, increment, 0)


def test_the_scene_is_immutable_and_has_five_numbers():
    scene = scene_from_scan(scan_at(0.0))
    assert [f.name for f in dataclasses.fields(Scene)] == ['left', 'right', 'front_axis', 'island_end', 'wall_heading']
    with pytest.raises(dataclasses.FrozenInstanceError):
        scene.left = 1.0


def test_the_walls_beside_and_ahead_at_the_start_pose():
    scene = scene_from_scan(scan_at(0.0))
    # the median of a +-25 deg window reads slightly more than the perpendicular distance (0.65 and 0.35)
    assert scene.left == pytest.approx(0.665, abs=0.01)       # island face at y = -0.5
    assert scene.right == pytest.approx(0.36, abs=0.01)       # outer wall at y = -1.5
    assert scene.front_axis == pytest.approx(1.5 - 0.128, abs=0.01)


@pytest.mark.parametrize('x', [-0.4, -0.1, 0.0, 0.2, 0.3])
def test_the_island_end_is_its_distance_ahead_of_base_link(x):
    scene = scene_from_scan(scan_at(x), direction='ccw')
    assert scene.island_end == pytest.approx(INNER - x, abs=0.02)


def test_the_island_end_is_unknown_when_it_is_beyond_the_view_or_already_passed():
    assert math.isinf(scene_from_scan(scan_at(-0.95)).island_end)     # still on the wall at the view limit
    assert math.isinf(scene_from_scan(scan_at(0.6)).island_end)       # past the end: no wall beside


def test_clockwise_looks_at_the_right_hand_wall():
    scan = scan_at(0.0, y=-1.15, yaw=math.pi)
    right_scene = scene_from_scan(scan, forward_yaw=0.0, direction='cw')
    assert right_scene.island_end == pytest.approx(0.5 - 0.0, abs=0.02)
    assert right_scene.right == pytest.approx(0.665, abs=0.01)         # the island is now on the right


def full_circle(scan, rays=504):
    """The same scan as the car's LD19 gives it: angles 0..2 pi (the lab and Gazebo give -110..+110 deg)."""
    src = np.array(scan.ranges)
    angles = scan.angle_min + np.arange(src.size) * scan.angle_increment
    out = np.full(rays, math.inf)
    for i in range(rays):
        a = math.atan2(math.sin(i * 2 * math.pi / rays), math.cos(i * 2 * math.pi / rays))
        k = int(round((a - scan.angle_min) / scan.angle_increment))
        if 0 <= k < src.size and abs(angles[k] - a) < scan.angle_increment:
            out[i] = src[k]
    return square_scan.make_scan(list(out), 0.0, 2 * math.pi / rays, 0)


@pytest.mark.parametrize('direction, yaw', [('ccw', 0.0), ('cw', math.pi)])
def test_a_full_circle_scan_gives_the_same_scene_in_both_directions(direction, yaw):
    """11 nov 2026: clockwise on the car (angles 0..2 pi) the island wall was never found: no island end and no wall
    heading in any clockwise round (late turns into the outer wall in narrow corridors)."""
    scan = scan_at(0.0, yaw=yaw)
    expected = scene_from_scan(scan, direction=direction)
    scene = scene_from_scan(full_circle(scan), direction=direction)
    assert math.isfinite(expected.island_end) and math.isfinite(expected.wall_heading)
    assert scene.island_end == pytest.approx(expected.island_end, abs=0.03)
    assert scene.wall_heading == pytest.approx(expected.wall_heading, abs=0.02)
    assert scene.left == pytest.approx(expected.left, abs=0.02) and scene.right == pytest.approx(expected.right, abs=0.02)


def test_a_few_lost_beams_do_not_move_the_island_end():
    scan = scan_at(0.2)
    clean = scene_from_scan(scan).island_end
    ranges = np.array(scan.ranges)
    wall = np.where(np.isfinite(ranges))[0]
    ranges[wall[::9]] = math.inf                                      # ~11 % dropouts, spread out
    scan.ranges = list(ranges)
    assert scene_from_scan(scan).island_end == pytest.approx(clean, abs=0.03)


def test_a_stretch_of_wall_with_no_return_is_not_the_island_end():
    """11 nov 2026, 22:28, the car: a 13 cm patch of the island wall (a panel joint) returned nothing, was read as the
    end 0.19 m ahead with the real one 0.7 m away, and the car turned into the island."""
    scan = scan_at(-0.2)                                              # the real end is 0.7 m ahead
    clean = scene_from_scan(scan).island_end
    ranges = np.array(scan.ranges)
    angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
    hole = (angles > math.radians(50)) & (angles < math.radians(62))  # 12 deg of the island wall, ~0.2-0.35 m ahead
    assert hole.sum() > 10
    ranges[hole] = math.inf
    scan.ranges = list(ranges)
    assert scene_from_scan(scan).island_end == pytest.approx(clean, abs=0.03)


def test_noise_keeps_the_island_end_within_a_few_centimetres():
    errors = [abs(scene_from_scan(scan_at(0.2, noise=0.008, seed=s)).island_end - 0.3) for s in range(20)]
    assert max(errors) < 0.06


@pytest.mark.parametrize('yaw', [-0.08, 0.0, 0.08])
def test_a_car_that_is_not_parallel_to_the_wall_still_finds_the_island_end(yaw):
    """Gazebo's heading wobbles by ~0.07 rad: the wall is a slanted line, not a constant lateral distance."""
    x = -0.2
    scene = scene_from_scan(scan_at(x, yaw=yaw), forward_yaw=0.0)
    expected = (INNER - x) * math.cos(yaw) + (-INNER + 1.15) * math.sin(yaw)
    assert scene.island_end == pytest.approx(expected, abs=0.03)


@pytest.mark.parametrize('yaw', [-0.3, -0.1, 0.0, 0.05, 0.2])
def test_the_wall_heading_is_the_car_yaw_relative_to_the_straight(yaw):
    scene = scene_from_scan(scan_at(-0.3, yaw=yaw))
    assert scene.wall_heading == pytest.approx(yaw, abs=0.02)


def test_the_wall_heading_with_lidar_noise_stays_within_a_degree():
    errors = [abs(scene_from_scan(scan_at(-0.3, yaw=0.1, noise=0.008, seed=s)).wall_heading - 0.1) for s in range(20)]
    assert max(errors) < math.radians(1.5)
