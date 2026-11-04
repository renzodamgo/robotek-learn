"""What the LiDAR tells the Open strategy: four numbers per scan, as one immutable value."""
import math
from dataclasses import dataclass

import numpy as np

from robotek_common.vehicle import LIDAR_OFFSET_X

SIDE_HALF = 0.44                  # rad around +-90 deg for the side walls
AXIS_HALF = math.radians(3.0)     # rad around the track axis for the front wall
END_FROM = math.radians(20.0)     # the island wall is followed from 90 deg down to this bearing
WALL_TOLERANCE = 0.03             # m: a beam is on the island wall within this of the fitted line
FIT_HALF = math.radians(20.0)     # the wall line is fitted to the beams within this of the side bearing
MAX_SLOPE = math.tan(0.30)        # rad: steeper than this is not a wall beside the car
LATERAL_RANGE = (0.15, 0.90)      # m: plausible distance from the car to the island wall
WALL_SIDE_RANGE = (0.12, 1.20)    # m: plausible distance to a side wall used for the heading
WALL_MAX_SLOPE = math.tan(math.radians(30.0))   # steeper than this is not the wall beside the car (heading)
MAX_GAP_BEAMS = 2                 # beams lost to dropouts that do not end the wall


@dataclass(frozen=True)
class Scene:
    left: float = math.inf         # median distance to the wall on the left (m)
    right: float = math.inf
    front_axis: float = math.inf   # distance to the wall ahead along the track axis
    island_end: float = math.inf   # x of the visible end of the island wall, base_link frame (m)
    wall_heading: float = math.inf  # car heading relative to the side walls (rad, + = left), inf when none fits


def scene_from_scan(scan, forward_yaw=0.0, direction='ccw'):
    """One LaserScan (ranges, angle_min, angle_increment, range_min, range_max) -> Scene."""
    angles, ranges, ok = scan_arrays(scan)
    sign = 1.0 if direction == 'ccw' else -1.0
    return Scene(
        left=_median_range(angles, ranges, ok, math.pi / 2),
        right=_median_range(angles, ranges, ok, -math.pi / 2),
        front_axis=_front_axis(angles, ranges, ok, forward_yaw),
        island_end=island_end(angles, ranges, ok, sign),
        wall_heading=wall_heading(angles, ranges, ok))


def _window(angles, ok, centre, half):
    relative = np.arctan2(np.sin(angles - centre), np.cos(angles - centre))
    return ok & (np.abs(relative) <= half)


def _median_range(angles, ranges, ok, centre):
    selected = _window(angles, ok, centre, SIDE_HALF)
    return float(np.median(ranges[selected])) if selected.any() else math.inf


def _front_axis(angles, ranges, ok, forward_yaw):
    selected = _window(angles, ok, forward_yaw, AXIS_HALF)
    if not selected.any():
        return math.inf
    return float(np.median(ranges[selected] * np.cos(angles[selected] - forward_yaw)))


def island_end(angles, ranges, ok, sign):
    """Where the island wall beside the car ends, found as the jump in range along the wall.

    A line is fitted to the wall beside the car (beams around 90 deg; the car is rarely parallel to it) and followed
    towards the front: the end is the last beam on the line before the range jumps. inf when the wall does not end
    inside the view (it continues to the front bearing limit) or there is no wall beside the car.
    """
    return island_wall(angles, ranges, ok, sign).end


@dataclass(frozen=True)
class IslandWall:
    """The island wall line in the LiDAR frame, island side positive (y = slope * x + offset), and its end."""
    slope: float = 0.0
    offset: float = math.inf       # inf: no island wall beside the car
    end: float = math.inf          # as Scene.island_end (base_link x)


def island_wall(angles, ranges, ok, sign):
    """The fitted island wall and its end (island_end's work, also drawn in RViz)."""
    bearing = sign * angles                                  # positive towards the island side
    side = ok & (np.abs(bearing - math.pi / 2) <= FIT_HALF)
    side &= ranges <= 1.5 * np.median(ranges[side]) if side.any() else side     # not beams past the wall's end
    if side.sum() < 10:
        return IslandWall()
    with np.errstate(invalid='ignore'):                      # lost beams are inf
        x, y = ranges * np.cos(bearing), ranges * np.sin(bearing)
    slope, offset = np.polyfit(x[side], y[side], 1)
    if abs(slope) > MAX_SLOPE or not LATERAL_RANGE[0] < offset < LATERAL_RANGE[1]:
        return IslandWall()                                  # no island wall beside the car
    order = np.argsort(-bearing)                             # from the side beams towards the front
    with np.errstate(invalid='ignore'):
        on_wall = ok & (np.abs(y - (slope * x + offset)) <= WALL_TOLERANCE)
    last = _last_on_wall(order[(bearing[order] <= math.pi / 2) & (bearing[order] >= END_FROM)], on_wall)
    if last is None or bearing[last] <= END_FROM + 0.05:
        return IslandWall(float(slope), float(offset))       # still on the wall at the limit: no end in view
    return IslandWall(float(slope), float(offset), float(x[last]) + LIDAR_OFFSET_X)


def wall_heading(angles, ranges, ok):
    """The car's heading relative to the straight walls beside it (rad, + = turned left), from a line fitted on each side.

    Each side uses the beams within FIT_HALF of +-90 deg (not past the wall's end); a side counts when its line is
    straight (median residual within WALL_TOLERANCE), not too steep and at a plausible distance. Both sides are averaged
    by their number of beams. inf when neither side fits (at a corner, or blind).
    """
    total, weight = 0.0, 0
    with np.errstate(invalid='ignore'):
        x, y = ranges * np.cos(angles), ranges * np.sin(angles)
    for side in (1.0, -1.0):
        beams = ok & (np.abs(np.arctan2(np.sin(angles - side * math.pi / 2),
                                        np.cos(angles - side * math.pi / 2))) <= FIT_HALF)
        if beams.sum() < 10:
            continue
        beams &= ranges <= 1.5 * np.median(ranges[beams])
        if beams.sum() < 10:
            continue
        slope, offset = np.polyfit(x[beams], y[beams], 1)
        if (abs(slope) > WALL_MAX_SLOPE or not WALL_SIDE_RANGE[0] < side * offset < WALL_SIDE_RANGE[1]
                or np.median(np.abs(y[beams] - (slope * x[beams] + offset))) > WALL_TOLERANCE):
            continue
        total += -math.atan(slope) * int(beams.sum())
        weight += int(beams.sum())
    return total / weight if weight else math.inf


def scan_arrays(scan):
    """angles, ranges and the valid-beam mask of a LaserScan (as scene_from_scan reads it)."""
    ranges = np.asarray(scan.ranges, dtype=float)
    angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
    ok = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    return angles, ranges, ok


def _last_on_wall(beams, on_wall):
    """The last beam on the wall before a run of more than MAX_GAP_BEAMS beams that are not."""
    last, missing = None, 0
    for i in beams:
        if on_wall[i]:
            last, missing = i, 0
        else:
            missing += 1
            if missing > MAX_GAP_BEAMS and last is not None:
                break
    return last
