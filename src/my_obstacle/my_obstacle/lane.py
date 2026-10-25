"""The field and the lane the car drives (step 1).

The field is a 3 x 3 m square with a 1 x 1 m island in the middle. The car drives the lane, the
rounded square halfway between the island and the outer wall: four 1 m straights joined by four
quarter circles of 0.5 m radius around the island's corners.

Everything is worked in one frame, the *track frame*: origin at the field centre, the start
straight at the bottom (y = -1), travel +x, always counter-clockwise. A clockwise round is the
same round seen in a mirror (x -> -x): the strategy mirrors what the LiDAR sees, the steering and
the colours (passing a red sign on the right in CW is passing it on the left in the mirror).

``progress`` counts 1 unit per straight and 1 per corner, 8 per lap, from the start of the
bottom straight: on straight k (0 bottom, 1 right, 2 top, 3 left) it runs from 2k to 2k + 1 and
the corner after it from 2k + 1 to 2k + 2. On a straight one unit is one metre; on a corner the
lane is 0.785 m long.

Signs sit only on the straights, at three seats each (both ends and the middle: progress 2k,
2k + 0.5 and 2k + 1) and in two rows, 0.4 m or 0.6 m from the island: 0.1 m to either side of the
lane centre.
"""
import math
from functools import lru_cache

import numpy as np

INNER = 0.5                    # m, half side of the island
OUTER = 1.5                    # m, half side of the field
UNITS_PER_LAP = 8.0
CORNER_UNITS = math.pi / 4.0   # m of lane in a corner (a 0.5 m quarter circle)
ROW = 0.1                      # m from the lane centre to a row of seats (+ island side, - outer side)
SEATS = tuple(2.0 * k + along for k in range(4) for along in (0.0, 0.5, 1.0))   # progress of the 12 seats


def wrap(angle):
    """An angle in (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    return max(low, min(high, value))


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
    """Unit vector to the left of travel: toward the island."""
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
    """``lane_pose`` for an array of progress values, without the heading: (x, y) arrays."""
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
    """Progress of the lane point nearest to (x, y), searching forward from ``start`` (a 2 cm grid)."""
    s = start + _offsets(span, step)
    lx, ly = _lane_points(s)
    return float(s[int(np.argmin(np.hypot(x - lx, y - ly)))])


def advance(progress, metres):
    """Progress after driving ``metres`` along the lane at the local scale (1 m per unit on a
    straight, 0.785 m per unit on a corner)."""
    section = int(progress) % 8
    scale = 1.0 if section % 2 == 0 else CORNER_UNITS
    return progress + metres / scale


def track_s(x, y):
    """Progress of any point of the field, lane or not: along the straight beside it, or the
    angle around the island corner it is in front of. Works on arrays."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    out = np.zeros(np.broadcast(x, y).shape)
    for k in range(4):
        c, s = math.cos(-k * math.pi / 2), math.sin(-k * math.pi / 2)
        xr, yr = c * x - s * y, s * x + c * y
        straight = (yr <= -INNER) & (np.abs(xr) <= INNER)
        corner = (xr > INNER) & (yr <= -INNER)
        out = np.where(straight, 2 * k + xr + INNER, out)
        angle = np.arctan2(yr + INNER, xr - INNER)
        out = np.where(corner, 2 * k + 1 + (angle + math.pi / 2) / (math.pi / 2), out)
    return out


def unwrap_s(previous, current):
    """``current`` (0 to 8) moved by whole laps to the value nearest ``previous``: progress that
    keeps counting laps."""
    return previous + ((current - previous + 4.0) % 8.0 - 4.0)
