import numpy as np

from robotek_course import vision


def _image():
    """Grey field (V 200), black wall in rows 20-39, a dark car part in the last 40 rows."""
    value = np.full((240, 320), 200, np.uint8)
    value[20:40] = 10
    value[200:] = 10
    value[120, :] = 50                     # a thin dark line on the field: not a wall
    return value


def test_wall_edges_find_the_lower_edge_of_the_wall():
    edges = vision.wall_edges(_image(), 200)
    assert (edges == 40).all()


def test_field_mask_keeps_only_the_field_inside_the_walls():
    value = _image()
    keep = vision.field_mask(value.shape, value, bottom_crop=40 / 240, wall_mask=True)
    assert not keep[:40].any()             # the wall and above
    assert keep[40:200].all()              # the field, the thin line included
    assert not keep[200:].any()            # the bottom cut (the car)


def test_field_mask_defaults_are_the_simulation_ones():
    value = _image()
    keep = vision.field_mask(value.shape, value, bottom_crop=0., wall_mask=False)
    assert not keep[:int(240 * .08)].any() and keep[int(240 * .08):].all()
