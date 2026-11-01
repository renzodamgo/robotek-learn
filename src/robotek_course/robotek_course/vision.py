"""Camera helper for step 8: how long are the blue and the orange lines in one image?

The vote (which direction, and how sure) is the exercise; measuring the lines is OpenCV work that is not.
The colour ranges are the simulation's (OpenCV HSV: h_min, h_max, s_min, v_min).

On the real car two extra cuts, set by car.launch through the environment (off in the simulation):
  - ROBOTEK_CAMERA_BOTTOM_CROP: share of rows at the bottom of the image to ignore (parts of the car in view: on
    the team's car the top of the LiDAR fills the bottom 10 %, and its LED looks blue);
  - ROBOTEK_CAMERA_WALL_MASK=1: only look at the field inside the black walls. For each column the lowest band of at
    least WALL_MIN_ROWS dark rows (scanning up from the bottom cut) is the wall; everything above its lower edge (the
    wall and the room behind it) is ignored. The edge is smoothed across columns so a thin dark line on the field
    is never taken for the wall.
"""
import os

import cv2
import numpy as np

BLUE_HSV = (90, 135, 60, 60)
ORANGE_HSV = (5, 28, 60, 60)
BOTTOM_CROP = float(os.environ.get('ROBOTEK_CAMERA_BOTTOM_CROP', '0'))
WALL_MASK = os.environ.get('ROBOTEK_CAMERA_WALL_MASK', '0') == '1'
WALL_V_MAX = 60          # HSV value: darker than this is the black wall
WALL_MIN_ROWS = 6        # a dark band at least this tall is the wall (the coloured lines are 1-3 px far away)
WALL_SMOOTH = 31         # columns of the median filter on the wall edge


def _mask(hsv, h_min, h_max, s_min, v_min):
    return cv2.inRange(hsv, np.array([h_min, s_min, v_min]), np.array([h_max, 255, 255]))


def _longest_line(mask):
    """Length in pixels of the longest elongated blob of the mask (a line, not a patch)."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    lengths = []
    for contour in contours:
        if cv2.contourArea(contour) < 25:
            continue
        _, dimensions, _ = cv2.minAreaRect(contour)
        short, long = sorted(dimensions)
        if long >= 3 * max(short, 1.0):
            lengths.append(long)
    return max(lengths, default=0.0)


def wall_edges(value, bottom):
    """Per column, the row just below the black wall: the lowest band of WALL_MIN_ROWS dark rows above `bottom`,
    median-smoothed across columns. 0 where no wall is found."""
    dark = (value[:bottom] < WALL_V_MAX).astype(np.int32)
    n = WALL_MIN_ROWS
    if dark.shape[0] <= n:
        return np.zeros(value.shape[1], dtype=np.int32)
    run = np.vstack([np.zeros((1, dark.shape[1]), np.int32), np.cumsum(dark, axis=0)])
    full = (run[n:] - run[:-n]) == n                     # full[y]: rows y..y+n-1 all dark
    last = full.shape[0] - 1 - np.argmax(full[::-1], axis=0)
    edges = np.where(full.any(axis=0), last + n, 0).astype(np.int32)
    k = WALL_SMOOTH // 2
    padded = np.pad(edges, k, mode='edge')
    return np.median(np.lib.stride_tricks.sliding_window_view(padded, 2 * k + 1), axis=1).astype(np.int32)


def field_mask(shape, value, bottom_crop=None, wall_mask=None):
    """255 where the colour search may look: below the top 8 % (or below the wall), above the bottom cut."""
    bottom_crop = BOTTOM_CROP if bottom_crop is None else bottom_crop
    wall_mask = WALL_MASK if wall_mask is None else wall_mask
    h, w = shape[:2]
    keep = np.full((h, w), 255, np.uint8)
    keep[:int(h * 0.08)] = 0                 # the very top of the image only holds the wall and the room
    bottom = h - int(h * bottom_crop)
    keep[bottom:] = 0
    if wall_mask:
        edges = wall_edges(value, bottom)
        rows = np.arange(h)[:, None]
        keep[(rows < edges[None, :]) & (edges[None, :] > 0)] = 0
    return keep


def line_lengths(bgr):
    """(blue_length, orange_length) in pixels for one BGR image (a numpy array)."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    kernel = np.ones((3, 5), dtype=np.uint8)
    keep = field_mask(bgr.shape, hsv[..., 2])
    lengths = []
    for colour in (BLUE_HSV, ORANGE_HSV):
        mask = cv2.morphologyEx(_mask(hsv, *colour), cv2.MORPH_CLOSE, kernel)
        mask[keep == 0] = 0
        lengths.append(_longest_line(mask))
    return tuple(lengths)
