"""Seeing the traffic signs of the Obstacle Challenge: LiDAR clusters + camera colours.

Given to you, like ``vision.py`` in the Open: it is image processing, not strategy.

* ``scan_clusters(scan)``: short isolated groups of LiDAR returns (a 50 mm sign seen from up to 2.5 m),
  walls excluded. Positions in the car frame (x ahead, y left, metres, from ``base_link``).
* ``color_blocks(bgr, hfov, pitch, yaw)``: red and green blobs of the camera image, each with the bearing
  (rad, + = left) at which it is seen from the camera.
* ``associate(blocks, clusters)``: pairs each coloured blob with the LiDAR cluster at the same bearing; a
  cluster claimed by both colours is dropped. The result is what the strategy receives in ``on_signs``:
  a list of ``dict(x, y, color)`` (plus diagnostics).

The colour thresholds were tuned on the simulated camera; on the real car they need re-tuning.
"""
import math

import cv2
import numpy as np

from robotek_common.vehicle import CAMERA_HFOV, CAMERA_OFFSET_X, CAMERA_PITCH, LIDAR_OFFSET_X

CLUSTER_RANGE = 2.5        # m; farther returns are ignored
CLUSTER_GAP = 0.07         # m between neighbouring returns of one group
CLUSTER_SIZE = (0.015, 0.12)   # m, diameter of a group that can be a sign
SIGN_HALF = 0.025          # m, half of a 50 mm sign: from its front face to its centre

# HSV (low, high) ranges per sign colour, tuned on the simulated camera.
COLOR_RANGES = {
    'red': [((0, 90, 45), (8, 255, 255)), ((170, 90, 45), (179, 255, 255))],
    'green': [((35, 70, 45), (85, 255, 255))],
}


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def scan_clusters(scan):
    """Short isolated groups of returns -> [dict(x, y, points, diameter_m)] in the car frame."""
    groups, group = [], []
    for i, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not scan.range_min < distance < CLUSTER_RANGE:
            if group:
                groups.append(group)
            group = []
            continue
        angle = scan.angle_min + i * scan.angle_increment
        point = (distance * math.cos(angle), distance * math.sin(angle))
        if group and math.dist(point, group[-1]) > CLUSTER_GAP:
            groups.append(group)
            group = []
        group.append(point)
    if group:
        groups.append(group)
    result = []
    for points in groups:
        if len(points) < 2:
            continue
        pts = np.array(points)
        diameter = math.hypot(np.ptp(pts[:, 0]), np.ptp(pts[:, 1]))
        if not CLUSTER_SIZE[0] <= diameter <= CLUSTER_SIZE[1]:
            continue
        x, y = pts.mean(axis=0)
        distance = math.hypot(x, y)
        # The LiDAR sees the face toward the car: move to the centre of the 50 mm sign.
        x += SIGN_HALF * x / distance
        y += SIGN_HALF * y / distance
        result.append(dict(x=float(x + LIDAR_OFFSET_X), y=float(y), points=len(points),
                           diameter_m=float(diameter)))
    return result


def color_blocks(bgr, hfov=CAMERA_HFOV, pitch=CAMERA_PITCH, yaw=0.0, ranges=COLOR_RANGES, min_area=30):
    """Red and green upright blobs -> [dict(color, bearing, box)]; bearing in rad, + = left."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    height, width = bgr.shape[:2]
    focal = width / (2 * math.tan(hfov / 2))
    blocks = []
    for color, bands in ranges.items():
        mask = cv2.inRange(hsv, np.array(bands[0][0]), np.array(bands[0][1]))
        for low, high in bands[1:]:
            mask |= cv2.inRange(hsv, np.array(low), np.array(high))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if cv2.contourArea(contour) < min_area or h < 8 or h < 1.05 * w:
                continue          # signs are taller than wide; floor patches are not
            u = (x + w / 2 - width / 2) / focal
            v = (y + h / 2 - height / 2) / focal
            bearing = math.atan2(-u, math.cos(pitch) - v * math.sin(pitch)) + yaw
            blocks.append(dict(color=color, bearing=bearing, box=(x, y, w, h)))
    return blocks


def associate(blocks, clusters, angular_gate=0.12):
    """Each coloured blob to the LiDAR cluster at its bearing; conflicting colours are dropped."""
    proposals = {}
    for block in blocks:
        matches = [(abs(_wrap(math.atan2(c['y'], c['x'] - CAMERA_OFFSET_X) - block['bearing'])), i)
                   for i, c in enumerate(clusters)]
        matches = [pair for pair in matches if pair[0] < angular_gate]
        if not matches:
            continue
        error, index = min(matches)
        proposals.setdefault(index, []).append((block, error))
    result = []
    for index, matches in proposals.items():
        if len({b['color'] for b, _ in matches}) != 1:
            continue
        block, error = min(matches, key=lambda pair: pair[1])
        result.append(dict(clusters[index], color=block['color'], angular_error=error, box=block['box']))
    return result
