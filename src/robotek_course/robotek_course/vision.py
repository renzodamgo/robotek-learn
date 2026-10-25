"""Camera helper for step 8: how long are the blue and the orange lines in one image?

The vote (which direction, and how sure) is the exercise; measuring the lines is OpenCV work that is not.
The colour ranges are the simulation's (OpenCV HSV: h_min, h_max, s_min, v_min).
"""
import cv2
import numpy as np

BLUE_HSV = (90, 135, 60, 60)
ORANGE_HSV = (5, 28, 60, 60)


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


def line_lengths(bgr):
    """(blue_length, orange_length) in pixels for one BGR image (a numpy array)."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    kernel = np.ones((3, 5), dtype=np.uint8)
    top = int(bgr.shape[0] * 0.08)           # the very top of the image only holds the wall and the room
    lengths = []
    for colour in (BLUE_HSV, ORANGE_HSV):
        mask = cv2.morphologyEx(_mask(hsv, *colour), cv2.MORPH_CLOSE, kernel)
        mask[:top] = 0
        lengths.append(_longest_line(mask))
    return tuple(lengths)
