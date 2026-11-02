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
    assert not keep[:40 + vision.WALL_MARGIN].any()     # the wall, above it and a small margin below
    assert keep[40 + vision.WALL_MARGIN:200].all()      # the field, the thin line included
    assert not keep[200:].any()            # the bottom cut (the car)


def test_field_mask_defaults_are_the_simulation_ones():
    value = _image()
    keep = vision.field_mask(value.shape, value, bottom_crop=0., wall_mask=False)
    assert not keep[:int(240 * .08)].any() and keep[int(240 * .08):].all()


def test_colour_mask_wraps_through_red_and_caps_the_value():
    hsv = np.array([[[170, 80, 100], [5, 80, 100], [100, 80, 100], [120, 20, 200], [120, 20, 90]]], np.uint8)
    orange = vision._mask(hsv, 155, 10, 25, 35)
    assert orange[0].tolist() == [255, 255, 0, 0, 0]
    blue = vision._mask(hsv, 95, 135, 15, 35, 125)
    assert blue[0].tolist() == [0, 0, 255, 0, 255]       # the bright mat (V 200) is not the dark blue line


def test_line_counter_counts_a_line_once_after_it_leaves_the_band():
    import cv2
    blank = np.full((240, 320, 3), 200, np.uint8)
    with_line = blank.copy()
    cv2.line(with_line, (20, 190), (300, 190), (200, 120, 60), 4)     # blue in BGR, inside the bottom band
    counter = vision.LineCounter()
    blue_hsv = vision.BLUE_HSV
    vision.BLUE_HSV = (95, 135, 60, 60)
    try:
        for frame in [blank] * 3 + [with_line] * 4 + [blank] * 2:
            counter.update(frame)
        assert counter.counts['blue'] == 0                         # not yet: still within the gap
        counter.update(blank)
        assert counter.counts['blue'] == 1
        for frame in [blank] * 5:
            counter.update(frame)
        assert counter.counts['blue'] == 1                         # counted once
    finally:
        vision.BLUE_HSV = blue_hsv
