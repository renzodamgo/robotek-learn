"""A tiny Obstacle field for the tests: walls, island, signs and lot as boxes, a LiDAR and a camera.

World frame of the rules (origin at the centre, start straight at the bottom). The car pose is base_link.
No noise unless asked: the tests check geometry, the course's lab checks the rest.
"""
import math
from types import SimpleNamespace

import numpy as np

from robotek_common.vehicle import CAMERA_OFFSET_X, LIDAR_OFFSET_X, REAL_CAMERA_HFOV, REAL_CAMERA_YAW

RAYS = 306
HALF_ANGLE = math.radians(110.0)
WALL = 0.02


def field_boxes(signs=(), lot=None):
    """Boxes (xmin, xmax, ymin, ymax): outer walls, island, signs (x, y, colour), lot limits."""
    boxes = [(-1.5 - WALL, 1.5 + WALL, -1.5 - WALL, -1.5), (-1.5 - WALL, 1.5 + WALL, 1.5, 1.5 + WALL),
             (-1.5 - WALL, -1.5, -1.5, 1.5), (1.5, 1.5 + WALL, -1.5, 1.5), (-0.5, 0.5, -0.5, 0.5)]
    boxes += [(x - 0.025, x + 0.025, y - 0.025, y + 0.025) for x, y, _ in signs]
    if lot:
        x0, x1 = lot
        boxes += [(x0 - 0.02, x0, -1.5, -1.3), (x1, x1 + 0.02, -1.5, -1.3)]
    return np.array(boxes)


def scan(x, y, yaw, boxes, stamp=0.0, rng=None):
    """A LaserScan-like object seen from base_link pose (x, y, yaw)."""
    ox, oy = x + LIDAR_OFFSET_X * math.cos(yaw), y + LIDAR_OFFSET_X * math.sin(yaw)
    increment = 2 * HALF_ANGLE / (RAYS - 1)
    angles = -HALF_ANGLE + increment * np.arange(RAYS) + yaw
    dx, dy = np.cos(angles), np.sin(angles)
    best = np.full(RAYS, np.inf)
    with np.errstate(divide='ignore', invalid='ignore'):
        for xmin, xmax, ymin, ymax in boxes:
            tx = np.stack([(xmin - ox) / dx, (xmax - ox) / dx])
            ty = np.stack([(ymin - oy) / dy, (ymax - oy) / dy])
            near = np.maximum(tx.min(axis=0), ty.min(axis=0))
            far = np.minimum(tx.max(axis=0), ty.max(axis=0))
            best = np.minimum(best, np.where((far >= near) & (near > 1e-6), near, np.inf))
    if rng is not None:
        best = best + rng.normal(0.0, 0.008, RAYS)
    sec = int(stamp)
    return SimpleNamespace(ranges=list(best), angle_min=-HALF_ANGLE, angle_increment=increment,
                           range_min=0.12, range_max=12.0,
                           header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=int(round((stamp - sec) * 1e9)))))


def camera(x, y, yaw, signs, max_range=2.0):
    """Signs in the real camera's view, in the car frame, as the node's pairing would give them."""
    out = []
    c, s = math.cos(yaw), math.sin(yaw)
    for sx, sy, colour in signs:
        dx, dy = sx - x, sy - y
        bx, by = c * dx + s * dy, -s * dx + c * dy
        bearing = math.atan2(by, bx - CAMERA_OFFSET_X)
        if 0.12 < math.hypot(bx - CAMERA_OFFSET_X, by) < max_range and \
                abs(bearing - REAL_CAMERA_YAW) <= REAL_CAMERA_HFOV / 2:
            out.append(dict(x=bx, y=by, color=colour))
    return out


def clusters(signs, x, y, yaw):
    """The LiDAR clusters of the signs within 2.5 m, in the car frame (centres)."""
    out = []
    c, s = math.cos(yaw), math.sin(yaw)
    for sx, sy, _ in signs:
        dx, dy = sx - x, sy - y
        bx, by = c * dx + s * dy, -s * dx + c * dy
        if bx > -0.3 and math.hypot(bx, by) < 2.5:
            out.append(dict(x=bx, y=by))
    return out


def run(strategy, signs, direction='ccw', seconds=20.0, until=None, lot=None, dt=0.02, servo_delay=0.06):
    """Drive a course strategy in the tiny field: 10 Hz scans, 6 fps camera, 50 Hz gyro and control, a bicycle
    with a servo delay. Returns the list of (time, x, y, yaw, speed) and the strategy. ``until(strategy)`` stops it."""
    from robotek_common.vehicle import WHEELBASE
    boxes = field_boxes(signs, lot)
    x, y, yaw = (-0.25, -1.0, 0.0) if direction == 'ccw' else (0.25, -1.0, math.pi)
    rate, queue, track, camera_time = 0.0, [0.0] * max(1, int(round(servo_delay / dt))), [], -1.0
    for tick in range(int(seconds / dt)):
        now = tick * dt
        if tick % 5 == 0:
            strategy.on_scan(scan(x, y, yaw, boxes, stamp=now))
            if now - camera_time >= 1 / 6 - 1e-9:
                camera_time = now
                strategy.on_signs(camera(x, y, yaw, signs), now)
        strategy.on_imu(now, rate + 0.006)                     # a gyroscope bias, as the real one
        speed, steer = strategy.control(now, dt)
        queue.append(steer)
        wheel = queue.pop(0)
        rate = speed * math.tan(wheel) / WHEELBASE
        rx, ry = x - WHEELBASE / 2 * math.cos(yaw), y - WHEELBASE / 2 * math.sin(yaw)
        yaw += rate * dt
        rx, ry = rx + speed * dt * math.cos(yaw), ry + speed * dt * math.sin(yaw)
        x, y = rx + WHEELBASE / 2 * math.cos(yaw), ry + WHEELBASE / 2 * math.sin(yaw)
        track.append((now, x, y, yaw, speed))
        if until is not None and until(strategy):
            break
    return track, strategy


def passes(track, signs):
    """For every sign the car went past: (colour, 'left' or 'right' of the car, closest distance from the car's
    centre). Red must be on the car's left (passed on its right), green on its right."""
    out = []
    for sx, sy, colour in signs:
        for (_, x0, y0, yaw0, _), (_, x1, y1, yaw1, _) in zip(track, track[1:]):
            a0 = math.cos(yaw0) * (sx - x0) + math.sin(yaw0) * (sy - y0)
            a1 = math.cos(yaw1) * (sx - x1) + math.sin(yaw1) * (sy - y1)
            if a0 > 0 >= a1 and math.hypot(sx - x1, sy - y1) < 0.5:
                side = math.cos(yaw1) * (sy - y1) - math.sin(yaw1) * (sx - x1)
                out.append((colour, 'left' if side > 0 else 'right', abs(side)))
    return out
