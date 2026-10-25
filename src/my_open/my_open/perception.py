"""Step 3 - Perception: from one LiDAR scan to the four numbers of a Scene."""
import math

import numpy as np

from robotek_course.minisim import LIDAR_X, Scene

SIDE_HALF = 0.44                  # rad around +-90 deg for the side walls
AXIS_HALF = math.radians(3.0)     # rad around the track axis for the wall ahead
FIT_HALF = math.radians(20.0)     # the island wall is fitted to the beams within this of the side bearing
END_FROM = math.radians(20.0)     # the island wall is followed from 90 deg down to this bearing
WALL_TOLERANCE = 0.03             # m: a beam is on the wall within this of the fitted line
MAX_GAP_BEAMS = 2                 # lost beams that do not end the wall
MAX_SLOPE = math.tan(0.30)        # a line steeper than this (rad) is not a wall beside the car
LATERAL_RANGE = (0.15, 0.90)      # m: plausible distance from the car to the island wall


def perceive(scan, forward_yaw=0.0, sign=1):
    """One scan -> Scene.

    ``forward_yaw``: the track direction seen from the car (rad), so the wall ahead is measured along the track, not
    along where the car happens to point. ``sign``: +1 when the island is on the left, -1 on the right.
    """
    ranges = np.asarray(scan.ranges, dtype=float)
    angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
    ok = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    return Scene(left=median_range(angles, ranges, ok, math.pi / 2),
                 right=median_range(angles, ranges, ok, -math.pi / 2),
                 front_axis=front_axis(angles, ranges, ok, forward_yaw),
                 island_end=island_end(angles, ranges, ok, sign))


def _near(angles, ok, centre, half):
    """Which beams are valid and within ``half`` radians of the bearing ``centre``."""
    relative = np.arctan2(np.sin(angles - centre), np.cos(angles - centre))
    return ok & (np.abs(relative) <= half)


def median_range(angles, ranges, ok, centre):
    """Distance to the wall at the bearing ``centre``: the MEDIAN of the beams around it (inf when there are none)."""
    chosen = _near(angles, ok, centre, SIDE_HALF)
    return float(np.median(ranges[chosen])) if chosen.any() else math.inf


def front_axis(angles, ranges, ok, forward_yaw):
    """Distance to the wall ahead measured ALONG the track: each beam's range times cos(its angle off the track)."""
    chosen = _near(angles, ok, forward_yaw, AXIS_HALF)
    if not chosen.any():
        return math.inf
    return float(np.median(ranges[chosen] * np.cos(angles[chosen] - forward_yaw)))


def island_end(angles, ranges, ok, sign):
    """x (m, in the car's frame) where the island wall beside the car ends, or inf when it does not end in view.

    Fit a straight line to the wall beside the car, follow it towards the front, and stop at the last beam on the
    line before the range jumps (a run of more than MAX_GAP_BEAMS beams off the line).
    """
    bearing = sign * angles                                  # positive towards the island side
    side = ok & (np.abs(bearing - math.pi / 2) <= FIT_HALF)
    if side.any():
        side &= ranges <= 1.5 * np.median(ranges[side])      # not the beams that already passed the wall's end
    if side.sum() < 10:
        return math.inf
    with np.errstate(invalid='ignore'):                      # lost beams are inf
        x, y = ranges * np.cos(bearing), ranges * np.sin(bearing)
    slope, offset = np.polyfit(x[side], y[side], 1)          # y = slope * x + offset
    if abs(slope) > MAX_SLOPE or not LATERAL_RANGE[0] < offset < LATERAL_RANGE[1]:
        return math.inf                                      # that is not an island wall beside the car
    with np.errstate(invalid='ignore'):
        on_wall = ok & (np.abs(y - (slope * x + offset)) <= WALL_TOLERANCE)
    last, missing = None, 0
    for i in np.argsort(-bearing):                           # from the side beams towards the front
        if not (END_FROM <= bearing[i] <= math.pi / 2):
            continue
        if on_wall[i]:
            last, missing = i, 0
        else:
            missing += 1
            if missing > MAX_GAP_BEAMS and last is not None:
                break
    if last is None or bearing[last] <= END_FROM + 0.05:
        return math.inf                                      # still on the wall at the limit of the view
    return float(x[last]) + LIDAR_X
