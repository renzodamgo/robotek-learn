"""Immutable observation of the Obstacle v3 mission and the known lane geometry.

The lane is the rounded square the car drives: four 1 m straights joined by four quarter
circles of 0.5 m radius. ``progress`` counts 1 unit per straight and 1 unit per corner
(8 per lap, as ``obstacle_field.track_s``). The lane is always run counter-clockwise;
CW rounds are mirrored before the mission sees them.
"""
import math
from dataclasses import dataclass
from functools import lru_cache

import numpy as np

INNER = 0.5
OUTER = 1.5
UNITS_PER_LAP = 8.0
CORNER_UNITS = math.pi / 4.0  # arc length of a 0.5 m quarter circle


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    return max(low, min(high, value))


@dataclass(frozen=True)
class Scene:
    inner: float                 # m to the island side
    outer: float                 # m to the outer wall
    front: float                 # m of free space ahead
    heading_error: float         # rad, car heading minus lane tangent
    progress: float              # track units since Start
    pillar: tuple | None         # (lateral m, distance m, colour or None)
    direction: str               # 'ccw' or 'cw'


def lane_pose(progress):
    """(x, y, heading) of the lane centre at a progress value (units, any sign)."""
    progress %= UNITS_PER_LAP
    section = int(progress)
    frac = progress - section
    if section % 2 == 0:
        x, y, heading = -INNER + frac, -1.0, 0.0
    else:
        theta = -math.pi / 2 + frac * math.pi / 2
        x, y = INNER + INNER * math.cos(theta), -INNER + INNER * math.sin(theta)
        heading = theta + math.pi / 2
    for _ in range(section // 2):
        x, y, heading = -y, x, heading + math.pi / 2
    return x, y, wrap(heading)


def lane_normal(progress):
    """Unit vector to the left of travel (the island side on a CCW lane)."""
    heading = lane_pose(progress)[2]
    return -math.sin(heading), math.cos(heading)


@lru_cache(maxsize=8)
def _offsets(span, step):
    offsets, offset = [], 0.0
    while offset <= span:
        offsets.append(offset)
        offset += step
    return np.array(offsets)


def _lane_points(progress):
    """Vectorised ``lane_pose`` without the heading: (x, y) arrays."""
    progress = np.mod(progress, UNITS_PER_LAP)
    section = progress.astype(int)
    frac = progress - section
    theta = -math.pi / 2 + frac * math.pi / 2
    straight = section % 2 == 0
    x = np.where(straight, -INNER + frac, INNER + INNER * np.cos(theta))
    y = np.where(straight, -1.0, -INNER + INNER * np.sin(theta))
    for turns in range(1, 4):
        mask = section // 2 == turns
        for _ in range(turns):
            x[mask], y[mask] = -y[mask], x[mask].copy()
    return x, y


def progress_of(x, y, start, span=3.0, step=0.02):
    """Lane progress of the point (x, y) nearest to ``start`` (searches forward only)."""
    s = start + _offsets(span, step)
    lx, ly = _lane_points(s)
    return float(s[int(np.argmin(np.hypot(x - lx, y - ly)))])


def advance(progress, metres):
    """Progress units covered by ``metres`` at the local scale (straights 1 m, corners 0.785 m)."""
    section = int(progress) % 8
    scale = 1.0 if section % 2 == 0 else CORNER_UNITS
    return progress + metres / scale
