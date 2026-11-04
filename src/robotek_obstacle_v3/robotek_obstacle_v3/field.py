"""Obstacle field geometry in the mirrored track frame (rules: 3 m square, 1 m island).

The round is always counter-clockwise in this frame and the car starts in the
bottom straight heading +x. CW rounds mirror lateral measurements, yaw and
steering and swap red/green (passing a red on the right in CW is passing it on
the left in the mirror). Sign seats sit at both ends and the middle of each
straight, 400/600 mm from the inner wall. Car outline and wheelbase match the URDF.
"""
import math

import numpy as np
from .car import (  # noqa: F401  (re-exported for the strategy modules)
    CAMERA_HFOV, CAMERA_OFFSET_X, CAMERA_PITCH, HALF_LENGTH as HALF_L, HALF_WIDTH as HALF_W,
    OUTLINE_FRONT as FRONT_L, OUTLINE_REAR as REAR_L,
    LIDAR_OFFSET_X, WHEELBASE)

OUTER = 1.5
INNER = .5
PILLAR_R = .0354  # half diagonal of a 50 mm pillar
# Straights in CCW order: tangent of travel and outward normal.
STRAIGHTS = [((1., 0.), (0., -1.)), ((0., 1.), (1., 0.)), ((-1., 0.), (0., 1.)), ((0., -1.), (-1., 0.))]
SWAP = {'red': 'green', 'green': 'red'}


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def track_s(x, y):
    """Lane-independent progress: 8 sections of 1 unit, CCW from the bottom straight's start.

    Straights use the along-track coordinate; corners the angle around the island corner.
    """
    x = np.asarray(x, dtype=float); y = np.asarray(y, dtype=float)
    out = np.zeros(np.broadcast(x, y).shape)
    for k in range(4):
        c, s = math.cos(-k*math.pi/2), math.sin(-k*math.pi/2)
        xr, yr = c*x-s*y, s*x+c*y
        straight = (yr <= -INNER) & (np.abs(xr) <= INNER)
        corner = (xr > INNER) & (yr <= -INNER)
        out = np.where(straight, 2*k+xr+INNER, out)
        angle = np.arctan2(yr+INNER, xr-INNER)
        out = np.where(corner, 2*k+1+(angle+math.pi/2)/(math.pi/2), out)
    return out


def track_heading(x, y):
    """Direction of travel (CCW) of the lane through (x, y): straight tangent or corner arc."""
    x = np.asarray(x, dtype=float); y = np.asarray(y, dtype=float)
    out = np.zeros(np.broadcast(x, y).shape)
    for k in range(4):
        c, s = math.cos(-k*math.pi/2), math.sin(-k*math.pi/2)
        xr, yr = c*x-s*y, s*x+c*y
        straight = (yr <= -INNER) & (np.abs(xr) <= INNER)
        corner = (xr > INNER) & (yr <= -INNER)
        out = np.where(straight, k*math.pi/2, out)
        out = np.where(corner, np.arctan2(yr+INNER, xr-INNER)+math.pi/2+k*math.pi/2, out)
    return out


def unwrap_s(previous, current):
    return previous+((current-previous+4.) % 8.-4.)


def make_seats():
    seats = []
    for k, (t, n) in enumerate(STRAIGHTS):
        for along in (-.5, 0., .5):
            for row in (.4, .6):
                seats.append(dict(id=len(seats), section=k, along=along, row=row, t=t, n=n,
                                  x=t[0]*along+n[0]*(INNER+row), y=t[1]*along+n[1]*(INNER+row)))
    return seats


def scan_points(scan, sign):
    """Valid lidar returns in the car frame (base_link), mirrored for CW (sign=-1)."""
    r = np.asarray(scan.ranges, dtype=float)
    a = scan.angle_min+np.arange(r.size)*scan.angle_increment
    ok = np.isfinite(r) & (r > .16) & (r < 4.)
    return r[ok]*np.cos(a[ok])+LIDAR_OFFSET_X, sign*r[ok]*np.sin(a[ok])
