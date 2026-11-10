"""Pose from lidar points fitted to the known walls, predicted with distance + gyro heading."""
import math

import numpy as np

from .field import INNER, OUTER, scan_points, wrap


class Localizer:
    """Odometry/IMU dead reckoning corrected by fitting lidar points to the known walls."""

    def __init__(self):
        self.x = self.y = self.th = 0.
        self.ready = False
        self.inliers = 0
        self.rms = math.nan

    def predict(self, distance, heading):
        mid = self.th + .5*wrap(heading-self.th)
        self.x += distance*math.cos(mid)
        self.y += distance*math.sin(mid)
        self.th = heading

    def _fit(self, bx, by, x, y, th, gate):
        c, s = math.cos(th), math.sin(th)
        px, py = x+c*bx-s*by, y+s*bx+c*by
        ax, ay = np.abs(px), np.abs(py)
        # Outer walls: the dominant axis gives the nearest wall from inside.
        onx = np.where(ax >= ay, np.sign(px), 0.)
        ony = np.where(ax < ay, np.sign(py), 0.)
        r_out = onx*px+ony*py-OUTER
        # Island walls: only the face a point is directly beside.
        inx = (ay <= INNER) & (ax > INNER-.05)
        iny = (ax <= INNER) & (ay > INNER-.05)
        nx = np.where(inx, np.sign(px), np.where(iny, 0., onx))
        ny = np.where(inx, 0., np.where(iny, np.sign(py), ony))
        r_in = np.where(inx, ax-INNER, np.where(iny, ay-INNER, np.inf))
        use_in = np.abs(r_in) < np.abs(r_out)
        nx = np.where(use_in, nx, onx)
        ny = np.where(use_in, ny, ony)
        r = np.where(use_in, r_in, r_out)
        keep = np.abs(r) < gate
        return px[keep], py[keep], nx[keep], ny[keep], r[keep]

    def correct(self, bx, by, gain=1., gate=.08, iterations=2, damping=2.):
        x, y, th = self.x, self.y, self.th
        for _ in range(iterations):
            px, py, nx, ny, r = self._fit(bx, by, x, y, th, gate)
            if r.size < 20:
                return False
            w = np.minimum(1., .02/np.maximum(np.abs(r), 1e-6))  # Huber
            J = np.stack([nx, ny, nx*-(py-y)+ny*(px-x)], axis=1)
            H = J.T@(J*w[:, None])+damping*np.eye(3)
            delta = -np.linalg.solve(H, J.T@(w*r))
            x, y, th = x+delta[0], y+delta[1], wrap(th+delta[2])
        self.inliers = int(r.size)
        self.rms = float(np.sqrt(np.mean(r**2)))
        self.x += gain*(x-self.x)
        self.y += gain*(y-self.y)
        self.th = wrap(self.th+gain*wrap(th-self.th))
        return True

    def initialize(self, bx, by, heading, guesses=None):
        """Global start in the bottom straight: try a few along-track guesses. guesses: [(x, y), ...] instead
        (the lot exit's dead-reckoned pose and its neighbours)."""
        best = None
        for x0, y0 in guesses or [(-.35, -1.), (0., -1.), (.35, -1.)]:
            self.x, self.y, self.th = x0, y0, heading
            for gate in (.45, .3, .2, .12, .08, .06):
                self.correct(bx, by, 1., gate, 2, .5)
            px, py, nx, ny, r = self._fit(bx, by, self.x, self.y, self.th, .05)
            score = r.size-2000*(float(np.sqrt(np.mean(r**2))) if r.size else 1.)
            if best is None or score > best[0]:
                best = (score, self.x, self.y, self.th)
        _, self.x, self.y, self.th = best
        self.ready = True


def direction_from_scan(scan):
    """Island on the left (CCW) or right (CW), assuming the car faces the driving direction.

    Fit the known walls under both hypotheses; the inlier fraction differs by ~0.2
    (the 1 m island versus the 3 m outer wall). Returns (sign, margin).
    """
    fractions = {}
    for sign in (1., -1.):
        bx, by = scan_points(scan, sign)
        if bx.size < 50:
            return 0., 0.
        loc = Localizer()
        loc.initialize(bx, by, 0.)
        fractions[sign] = loc._fit(bx, by, loc.x, loc.y, loc.th, .05)[4].size/bx.size
    sign = max(fractions, key=fractions.get)
    return sign, fractions[sign]-fractions[-sign]
