"""Travelled distance from consecutive LiDAR scans (no wheel encoder needed).

The real car has no encoder, so both strategies measure how far it moved by
aligning each scan to the previous one: rotation comes from the IMU yaw change,
translation from a point-to-line least-squares fit (walls are straight). When a
scan cannot be aligned (too few matched points) the distance falls back to the
commanded speed integrated through a simple acceleration-limited motor model.
"""
import math

import numpy as np

from .vehicle import LIDAR_OFFSET_X


def scan_points(msg, offset_x=LIDAR_OFFSET_X, max_range=4.0, min_range=0.16):
    r = np.asarray(msg.ranges, dtype=float)
    a = msg.angle_min + np.arange(r.size)*msg.angle_increment
    ok = np.isfinite(r) & (r > max(min_range, msg.range_min)) & (r < max_range)
    return np.stack([r[ok]*np.cos(a[ok])+offset_x, r[ok]*np.sin(a[ok])], axis=1)


class LidarOdometry:
    def __init__(self, accel=2.0, gate=0.08, min_matches=40, iterations=4):
        self.accel = accel
        self.gate = gate
        self.min_matches = min_matches
        self.iterations = iterations
        self.prev = None          # previous scan points (base frame)
        self.prev_normals = None
        self.prev_yaw = None
        self.speed_model = 0.0    # m/s, acceleration-limited commanded speed
        self.pending_model = 0.0  # model distance since the last scan
        self.delta = 0.0          # distance since last read
        self.measured = 0.0       # part of it from aligned scans only (no model fallback)
        self.last_quality = None  # 'lidar' | 'model'
        self.matches = 0

    # ---------------------------------------------------------------- model
    def command(self, speed, dt):
        """Integrate the commanded speed through the acceleration limit (fallback)."""
        step = self.accel*max(0.0, dt)
        self.speed_model += max(-step, min(step, speed-self.speed_model))
        self.pending_model += abs(self.speed_model)*max(0.0, dt)

    # ----------------------------------------------------------------- scan
    @staticmethod
    def _normals(points):
        """Line normal at each point from its neighbours along the scan."""
        if len(points) < 3:
            return np.zeros_like(points)
        d = np.roll(points, -1, axis=0)-np.roll(points, 1, axis=0)
        n = np.stack([-d[:, 1], d[:, 0]], axis=1)
        norm = np.linalg.norm(n, axis=1, keepdims=True)
        return n/np.maximum(norm, 1e-9)

    def scan(self, msg, yaw):
        points = scan_points(msg)
        moved = None
        if self.prev is not None and len(points) >= self.min_matches:
            dth = math.atan2(math.sin(yaw-self.prev_yaw), math.cos(yaw-self.prev_yaw))
            moved = self._align(points, dth)
        if moved is None:
            step = self.pending_model
            self.last_quality = 'model'
        else:
            step = moved
            self.measured += moved
            self.last_quality = 'lidar'
        self.pending_model = 0.0
        self.delta += step
        self.prev, self.prev_yaw = points, yaw
        self.prev_normals = self._normals(points)

    def _align(self, points, dth):
        """Translation of the car between scans (previous frame), rotation fixed by the IMU."""
        c, s = math.cos(dth), math.sin(dth)
        rotated = points@np.array([[c, s], [-s, c]])
        t = np.zeros(2)
        # Initial guess: straight ahead by the model distance.
        t[0] = self.pending_model
        prev, normals = self.prev, self.prev_normals
        for _ in range(self.iterations):
            moved = rotated+t
            # Nearest previous point (brute force: ~400 x 400).
            d2 = ((moved[:, None, :]-prev[None, :, :])**2).sum(axis=2)
            j = np.argmin(d2, axis=1)
            near = np.sqrt(d2[np.arange(len(moved)), j])
            keep = near < self.gate
            self.matches = int(keep.sum())
            if self.matches < self.min_matches:
                return None
            n = normals[j[keep]]
            r = ((moved[keep]-prev[j[keep]])*n).sum(axis=1)
            A = n.T@n+1e-6*np.eye(2)
            t = t-np.linalg.solve(A, n.T@r)
        # Both wall directions are needed to observe the along-track motion.
        eig = np.linalg.eigvalsh(n.T@n)
        if eig[0] < 0.05*self.matches:
            return None
        return float(np.hypot(*t))

    def read(self):
        """Distance since the previous read (consumed)."""
        d, self.delta = self.delta, 0.0
        return d

    def read_measured(self):
        """Distance from aligned scans only since its previous read (consumed)."""
        d, self.measured = self.measured, 0.0
        return d
