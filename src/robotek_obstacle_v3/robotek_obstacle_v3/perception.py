"""LiDAR sign clusters as Obstacle v3 sees them: WRO-Strategies' obstacle_perception.scan_clusters.

The same code as robotek_course.obstacle_vision.scan_clusters, but with the LiDAR offset of car.py (v3's geometry).
"""
import math

import numpy as np

from .car import LIDAR_OFFSET_X


def scan_clusters(scan):
    """Extract short isolated scan components; wall segments are excluded."""
    groups, group = [], []
    for i, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not scan.range_min < distance < 2.5:
            if group:
                groups.append(group)
            group = []
            continue
        angle = scan.angle_min + i * scan.angle_increment
        point = (distance * math.cos(angle), distance * math.sin(angle))
        if group and math.dist(point, group[-1]) > .07:
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
        if not .015 <= diameter <= .12:
            continue
        x, y = pts.mean(axis=0)
        distance = math.hypot(x, y)
        # Correct the observed front surface toward the 5 cm pillar center.
        x += .025 * x / distance
        y += .025 * y / distance
        result.append(dict(x=float(x + LIDAR_OFFSET_X), y=float(y), points=len(points),
                           diameter_m=diameter))
    return result
