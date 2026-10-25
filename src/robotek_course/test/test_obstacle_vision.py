"""Seeing the signs: colour blobs with their bearing, and their association with LiDAR clusters."""
import math

import numpy as np

from robotek_course.obstacle_vision import associate, color_blocks


def _image_with(color_bgr, column):
    image = np.full((480, 640, 3), 200, dtype=np.uint8)       # a grey floor
    image[200:300, column - 15:column + 15] = color_bgr         # an upright sign, taller than wide
    return image


def test_a_red_sign_on_the_right_has_a_negative_bearing():
    blocks = color_blocks(_image_with((55, 39, 238), 480), hfov=math.pi / 3, pitch=0.0)
    assert [b['color'] for b in blocks] == ['red']
    assert blocks[0]['bearing'] < 0.0


def test_a_green_sign_in_the_middle_is_straight_ahead():
    blocks = color_blocks(_image_with((44, 214, 68), 320), hfov=math.pi / 3, pitch=0.0)
    assert [b['color'] for b in blocks] == ['green']
    assert abs(blocks[0]['bearing']) < 0.02


def test_association_pairs_a_blob_with_the_cluster_at_its_bearing():
    blocks = [dict(color='green', bearing=0.0, box=(0, 0, 1, 2))]
    clusters = [dict(x=1.0, y=0.0), dict(x=1.0, y=0.6)]
    paired = associate(blocks, clusters)
    assert len(paired) == 1 and paired[0]['y'] == 0.0 and paired[0]['color'] == 'green'


def test_a_cluster_claimed_by_both_colours_is_dropped():
    blocks = [dict(color='green', bearing=0.0, box=(0, 0, 1, 2)), dict(color='red', bearing=0.01, box=(0, 0, 1, 2))]
    assert associate(blocks, [dict(x=1.0, y=0.0)]) == []
