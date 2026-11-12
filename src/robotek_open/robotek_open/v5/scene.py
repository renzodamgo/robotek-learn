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
MAX_SLOPE = math.tan(math.radians(30.0))   # steeper than this is not a wall beside the car
LATERAL_RANGE = (0.15, 0.90)      # m: plausible distance from the car to the island wall
MAX_GAP_BEAMS = 2                 # beams off the wall line that do not end the wall (noise)


@dataclass(frozen=True)
class Scene:
    left: float = math.inf         # median distance to the wall on the left (m)
    right: float = math.inf
    front_axis: float = math.inf   # distance to the wall ahead along the track axis
    island_end: float = math.inf   # x of the visible end of the island wall, base_link frame (m)
    wall_heading: float = math.inf  # car heading relative to the island wall (rad, + = left), inf without one


def scene_from_scan(scan, forward_yaw=0.0, direction='ccw'):
    """One LaserScan (ranges, angle_min, angle_increment, range_min, range_max) -> Scene."""
    angles, ranges, ok = scan_arrays(scan)
    sign = 1.0 if direction == 'ccw' else -1.0
    wall = island_wall(angles, ranges, ok, sign)
    return Scene(
        left=_median_range(angles, ranges, ok, math.pi / 2),
        right=_median_range(angles, ranges, ok, -math.pi / 2),
        front_axis=_front_axis(angles, ranges, ok, forward_yaw),
        island_end=wall.end,
        # The island wall's line gives the heading too (a separate fit of both side walls gave the same rounds with
        # the gyro reading 6 % short; 12 nov 2026 lab).
        wall_heading=-sign * math.atan(wall.slope) if math.isfinite(wall.offset) else math.inf)


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


@dataclass(frozen=True)
class IslandWall:
    """The island wall line in the LiDAR frame, island side positive (y = slope * x + offset), and its end."""
    slope: float = 0.0
    offset: float = math.inf       # inf: no island wall beside the car
    end: float = math.inf          # as Scene.island_end (base_link x)


def island_wall(angles, ranges, ok, sign):
    """The fitted island wall and where it ends, found as the jump in range along the wall (also drawn in RViz).

    A line is fitted to the wall beside the car (beams around 90 deg; the car is rarely parallel to it) and followed
    towards the front: the end is the last beam on the line before the range jumps. end is inf when the wall does not
    end inside the view (it continues to the front bearing limit); offset is inf when there is no wall beside the car.
    """
    # Positive towards the island side, wrapped to +-pi: the car's LD19 gives 0..2 pi, and clockwise (sign -1) that
    # is -2 pi..0, where no beam fell in the window around +90 deg: no island wall, no end and no wall heading in any
    # clockwise round on the car until 11 nov 2026 (the lab's scan is +-110 deg and hid it).
    bearing = np.arctan2(np.sin(sign * angles), np.cos(sign * angles))
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
    last = _last_on_wall(order[(bearing[order] <= math.pi / 2) & (bearing[order] >= END_FROM)], on_wall, ok)
    if last is None or bearing[last] <= END_FROM + 0.05:
        return IslandWall(float(slope), float(offset))       # still on the wall at the limit: no end in view
    return IslandWall(float(slope), float(offset), float(x[last]) + LIDAR_OFFSET_X)


def scan_arrays(scan):
    """angles, ranges and the valid-beam mask of a LaserScan (as scene_from_scan reads it)."""
    ranges = np.asarray(scan.ranges, dtype=float)
    angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
    ok = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    return angles, ranges, ok


def _last_on_wall(beams, on_wall, ok):
    """The last beam on the wall before a run of more than MAX_GAP_BEAMS beams that see past it.

    Only beams with a return count against the wall: past a real end the LiDAR sees what is behind (the far wall). A
    stretch with no return at all is a hole in the data, not the end: on the car a 13 cm patch of the island wall (the
    joint of two panels) returned nothing, read as the end 0.19 m ahead with the real one 0.7 m away, and the car
    turned into the island (11 nov 2026, 22:28).
    """
    last, beyond = None, 0
    for i in beams:
        if on_wall[i]:
            last, beyond = i, 0
        elif ok[i]:
            beyond += 1
            if beyond > MAX_GAP_BEAMS and last is not None:
                break
    return last
