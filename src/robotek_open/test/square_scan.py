"""Hand-built LD19 scans of the standard square field (1.5 m outer half width, island inside)."""
import math
from types import SimpleNamespace

import numpy as np

from robotek_open.open_constants import CONTROL_RATE, HARDWARE

DT = 1.0 / CONTROL_RATE
OUTER = 1.5


def raycast(x, y, yaw, inner, rng, noise=0.015):
    """LD19-like scan from the lidar pose: +-110 deg, 306 samples, nothing closer than 0.12 m."""
    ox = x + HARDWARE.lidar_offset_x * math.cos(yaw)
    oy = y + HARDWARE.lidar_offset_x * math.sin(yaw)
    count = 306
    angle_min = -math.radians(110.0)
    increment = -2.0 * angle_min / (count - 1)
    angles = angle_min + increment * np.arange(count)
    dx, dy = np.cos(angles + yaw), np.sin(angles + yaw)
    best = np.full(count, np.inf)
    for half, outside in ((OUTER, True), (inner, False)):
        for axis, sign in ((0, 1), (0, -1), (1, 1), (1, -1)):
            d, other_d = (dx, dy) if axis == 0 else (dy, dx)
            o, other_o = (ox, oy) if axis == 0 else (oy, ox)
            with np.errstate(divide='ignore', invalid='ignore'):
                t = (sign * half - o) / d
            hit_other = other_o + t * other_d
            valid = (t > 1e-6) & (np.abs(hit_other) <= half + 1e-9)
            if not outside:          # the island is only hit from outside it
                valid &= (np.abs(o) >= half) | (np.abs(other_o) >= half)
            best = np.where(valid & (t < best), t, best)
    ranges = best + rng.normal(0.0, noise, count)
    ranges[(ranges < 0.12) | ~np.isfinite(best)] = np.inf
    return ranges, angle_min, increment


def make_scan(ranges, angle_min, increment, tick):
    seconds = tick * DT
    return SimpleNamespace(
        ranges=list(ranges), angle_min=angle_min, angle_increment=increment, range_min=0.03,
        range_max=12.0, header=SimpleNamespace(stamp=SimpleNamespace(
            sec=int(seconds), nanosec=int(round((seconds - int(seconds)) * 1e9)))))
