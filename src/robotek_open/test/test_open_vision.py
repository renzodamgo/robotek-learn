"""Check visual ambiguity, independent-frame confirmation and the floor mask."""

from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from robotek_open.open_vision import OpenVision


def frame(blue_length, orange_length):
    """Draw elongated lines against a neutral floor."""
    image = np.full((120, 240, 3), 220, dtype=np.uint8)
    if blue_length:
        cv2.line(image, (10, 50), (10 + blue_length, 50), (255, 0, 0), 5)
    if orange_length:
        cv2.line(image, (10, 90), (10 + orange_length, 90), (0, 128, 255), 5)
    return image


@pytest.mark.parametrize('blue,orange,sign', [(180, 80, 1), (80, 180, -1)])
def test_requires_five_independent_agreeing_frames(blue, orange, sign):
    vision = OpenVision()
    image = frame(blue, orange)
    for index in range(4):
        vision.update(image, index * 0.1)
        assert vision.direction == 0
    # Repeated delivery of the same camera stamp is not another observation.
    for _ in range(10):
        vision.update(image, 0.3)
    assert vision.direction == 0
    vision.update(image, 0.4)
    assert vision.direction == sign


def test_real_camera_requires_three_independent_agreeing_frames():
    vision = OpenVision(confirmations=3)
    image = frame(180, 80)
    vision.update(image, 0.)
    vision.update(image, 1./6.)
    assert vision.direction == 0
    for _ in range(10):
        vision.update(image, 1./6.)
    assert vision.direction == 0
    vision.update(image, 2./6.)
    assert vision.direction == 1


@pytest.mark.parametrize('blue,orange', [(0, 0), (150, 150), (15, 0)])
def test_missing_balanced_or_short_lines_do_not_choose(blue, orange):
    vision = OpenVision()
    for index in range(10):
        vision.update(frame(blue, orange), index * 0.1)
    assert vision.direction == 0


def walled_frame():
    """White floor below a black wall band, with an orange-like wall of the room above it."""
    image = np.full((240, 320, 3), 220, dtype=np.uint8)
    image[:60] = (90, 170, 230)           # beige room: inside the simulation's orange range
    image[60:90] = (10, 10, 10)           # black field wall
    cv2.line(image, (20, 200), (300, 200), (255, 0, 0), 5)   # blue line on the floor
    return image


def test_floor_mask_ignores_the_room_above_the_wall():
    image = walled_frame()
    plain = OpenVision()
    plain.update(image, 0.0)
    assert plain.orange_length > plain.blue_length    # the room wins without the mask
    vision = OpenVision(floor_mask=True)
    vision.update(image, 0.0)
    assert vision.orange_length == 0.0 and vision.blue_length > 200
    assert (vision.floor_rows(cv2.cvtColor(image, cv2.COLOR_BGR2HSV)) == 90).all()


def test_floor_mask_keeps_thin_dark_lines_and_columns_without_wall():
    image = np.full((240, 320, 3), 220, dtype=np.uint8)
    cv2.line(image, (0, 120), (319, 120), (10, 10, 10), 3)    # printed line, not a wall
    vision = OpenVision(floor_mask=True)
    assert (vision.floor_rows(cv2.cvtColor(image, cv2.COLOR_BGR2HSV)) == 0).all()


def test_orange_range_can_wrap_through_red():
    image = np.full((120, 240, 3), 220, dtype=np.uint8)
    cv2.line(image, (10, 90), (200, 90), (60, 40, 200), 5)   # red-pink: H ~ 174
    assert OpenVision().orange_length == 0.0
    sim = OpenVision(); sim.update(image, 0.0)
    real = OpenVision(orange_hsv=(158, 4, 60, 40)); real.update(image, 0.0)
    assert sim.orange_length == 0.0 and real.orange_length > 150
