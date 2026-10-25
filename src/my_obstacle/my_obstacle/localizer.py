"""Where the car is (step 2): the known walls fitted to the LiDAR, predicted with distance + gyro.

The real car has no wheel encoder. Between scans the pose is *predicted* with the distance of the
LiDAR odometry and the heading of the gyroscope; on every scan it is *corrected* by fitting the
scan points to the walls the field is known to have (the outer square and the island), a few
Gauss-Newton iterations with a Huber weight so that signs and noise do not pull it. The error is
about a centimetre, which is what three laps without a map need.

At Start the car does not know its position along the start straight nor the direction of the
round: ``initialize`` tries three positions along the straight and keeps the best fit, and
``direction_from_scan`` fits the walls with the island on the left (CCW) and on the right (CW).
"""
import math

import numpy as np

from robotek_common.vehicle import LIDAR_OFFSET_X

from .lane import INNER, OUTER, wrap


def scan_points(scan, sign):
    """Valid LiDAR returns as points of the car frame (x ahead, y left), mirrored for CW (sign = -1)."""
    r = np.asarray(scan.ranges, dtype=float)
    a = scan.angle_min + np.arange(r.size) * scan.angle_increment
    ok = np.isfinite(r) & (r > 0.16) & (r < 4.0)
    return r[ok] * np.cos(a[ok]) + LIDAR_OFFSET_X, sign * r[ok] * np.sin(a[ok])


class Localizer:
    """Dead reckoning (distance + heading) corrected by fitting the scan to the known walls."""

    def __init__(self):
        self.x = self.y = self.th = 0.0
        self.ready = False
        self.inliers = 0
        self.rms = math.nan

    def predict(self, distance, heading):
        """Move ``distance`` metres along the mean of the old and new heading."""
        mid = self.th + 0.5 * wrap(heading - self.th)
        self.x += distance * math.cos(mid)
        self.y += distance * math.sin(mid)
        self.th = heading

    def _fit(self, bx, by, x, y, th, gate):
        """Scan points near a known wall: their positions, the wall normals and the residuals."""
        c, s = math.cos(th), math.sin(th)
        px, py = x + c * bx - s * by, y + s * bx + c * by
        ax, ay = np.abs(px), np.abs(py)
        # Outer walls: the dominant axis gives the nearest wall from inside.
        onx = np.where(ax >= ay, np.sign(px), 0.0)
        ony = np.where(ax < ay, np.sign(py), 0.0)
        r_out = onx * px + ony * py - OUTER
        # Island walls: only the face a point is directly beside.
        inx = (ay <= INNER) & (ax > INNER - 0.05)
        iny = (ax <= INNER) & (ay > INNER - 0.05)
        nx = np.where(inx, np.sign(px), np.where(iny, 0.0, onx))
        ny = np.where(inx, 0.0, np.where(iny, np.sign(py), ony))
        r_in = np.where(inx, ax - INNER, np.where(iny, ay - INNER, np.inf))
        use_in = np.abs(r_in) < np.abs(r_out)
        nx = np.where(use_in, nx, onx)
        ny = np.where(use_in, ny, ony)
        r = np.where(use_in, r_in, r_out)
        keep = np.abs(r) < gate
        return px[keep], py[keep], nx[keep], ny[keep], r[keep]

    def correct(self, bx, by, gain=1.0, gate=0.08, iterations=2, damping=2.0):
        """Pull the pose toward the wall fit by ``gain`` (1 = all the way). False if too few points."""
        x, y, th = self.x, self.y, self.th
        for _ in range(iterations):
            px, py, nx, ny, r = self._fit(bx, by, x, y, th, gate)
            if r.size < 20:
                return False
            w = np.minimum(1.0, 0.02 / np.maximum(np.abs(r), 1e-6))  # Huber
            J = np.stack([nx, ny, nx * -(py - y) + ny * (px - x)], axis=1)
            H = J.T @ (J * w[:, None]) + damping * np.eye(3)
            delta = -np.linalg.solve(H, J.T @ (w * r))
            x, y, th = x + delta[0], y + delta[1], wrap(th + delta[2])
        self.inliers = int(r.size)
        self.rms = float(np.sqrt(np.mean(r ** 2)))
        self.x += gain * (x - self.x)
        self.y += gain * (y - self.y)
        self.th = wrap(self.th + gain * wrap(th - self.th))
        return True

    def initialize(self, bx, by, heading):
        """Global start in the bottom straight: try a few along-track guesses, keep the best fit."""
        best = None
        for x0 in (-0.35, 0.0, 0.35):
            self.x, self.y, self.th = x0, -1.0, heading
            for gate in (0.45, 0.3, 0.2, 0.12, 0.08, 0.06):
                self.correct(bx, by, 1.0, gate, 2, 0.5)
            px, py, nx, ny, r = self._fit(bx, by, self.x, self.y, self.th, 0.05)
            score = r.size - 2000 * (float(np.sqrt(np.mean(r ** 2))) if r.size else 1.0)
            if best is None or score > best[0]:
                best = (score, self.x, self.y, self.th)
        _, self.x, self.y, self.th = best
        self.ready = True


def direction_from_scan(scan):
    """Island on the left (CCW, +1) or on the right (CW, -1), assuming the car faces the way it
    will drive. Fits the walls under both hypotheses: the fraction of points on a wall differs by
    about 0.2 (the 1 m island against the 3 m outer wall). Returns (sign, margin)."""
    fractions = {}
    for sign in (1.0, -1.0):
        bx, by = scan_points(scan, sign)
        if bx.size < 50:
            return 0.0, 0.0
        loc = Localizer()
        loc.initialize(bx, by, 0.0)
        fractions[sign] = loc._fit(bx, by, loc.x, loc.y, loc.th, 0.05)[4].size / bx.size
    sign = max(fractions, key=fractions.get)
    return sign, fractions[sign] - fractions[-sign]
