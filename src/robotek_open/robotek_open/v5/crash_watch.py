"""Crash detector for the car: keeps the last seconds of the round and says when the car has hit something.

Two signals (11 nov 2026, eight rounds with the car's LD19 and IMU):

* blind LiDAR: against a wall the LD19 sees almost nothing (under ~120 valid rays out of 504; 270-300 when driving).
  Every crash of that day showed >= 0.4 s of such scans while the car was asked to move; the record round, none
  (its lowest: 151). Pressed against a wall the count wanders 120-180, so a blind run only ends at clear_rays.
* gyro jump: a change of the yaw rate within 0.1 s bigger than any corner gives (corners reach 1.7-2.4 rad/s).

On a crash the node stops, resets the board and writes `window()` to a JSON file (see `dump`).
"""
from collections import deque
import json
import math


class CrashWatch:
    def __init__(self, window_s=5.0, min_rays=120, clear_rays=180, blind_s=0.4, gyro_jump=3.0, min_speed=0.2):
        self.window_s, self.min_rays, self.clear_rays, self.blind_s = window_s, min_rays, clear_rays, blind_s
        self.gyro_jump, self.min_speed = gyro_jump, min_speed
        self.ticks, self.scans = deque(), deque()
        self._blind_since = None
        self._gyro = deque()          # (t, gyro_z) over the last 0.1 s
        self.reason = None

    @staticmethod
    def valid_rays(ranges, range_min, range_max):
        return sum(1 for r in ranges if r is not None and math.isfinite(r) and range_min < r < range_max)

    def _trim(self, rows, t):
        while rows and t - rows[0]['t'] > self.window_s:
            rows.popleft()

    def scan(self, t, msg_fields, moving):
        """One LiDAR scan: msg_fields has angle_min, angle_increment, range_min, range_max, ranges. `moving` = the
        mission asks for speed above min_speed. Returns the crash reason or None."""
        rays = self.valid_rays(msg_fields['ranges'], msg_fields['range_min'], msg_fields['range_max'])
        self.scans.append({'t': t, 'valid': rays, **msg_fields, 'ranges': [round(r, 3) if r is not None and math.isfinite(r) else None
                                                                          for r in msg_fields['ranges']]})
        self._trim(self.scans, t)
        if not moving or rays >= self.clear_rays:
            self._blind_since = None      # hysteresis: a scan of 120-180 rays in a blind run does not end it
        elif rays < self.min_rays and self._blind_since is None:
            self._blind_since = t
        if self._blind_since is not None:
            if self.blind_s > 0.0 and t - self._blind_since >= self.blind_s and self.reason is None:
                self.reason = f'blind LiDAR {t - self._blind_since:.2f} s (< {self.min_rays} valid rays)'
        return self.reason

    def tick(self, t, row, gyro_z, moving):
        """One control tick: row = what the node commanded and saw. Returns the crash reason or None."""
        self.ticks.append({'t': t, **row})
        self._trim(self.ticks, t)
        self._gyro.append((t, gyro_z))
        while self._gyro and t - self._gyro[0][0] > 0.1:
            self._gyro.popleft()
        if self.gyro_jump > 0.0 and moving and self.reason is None:
            jump = max(abs(gyro_z - g) for _, g in self._gyro)
            if jump >= self.gyro_jump:
                self.reason = f'gyro jump {jump:.2f} rad/s in 0.1 s'
        return self.reason

    def window(self):
        return {'reason': self.reason, 'window_s': self.window_s,
                'ticks': list(self.ticks), 'scans': list(self.scans)}

    def dump(self, path, extra=None):
        data = self.window()
        data.update(extra or {})
        with open(path, 'w') as out:
            json.dump(data, out, allow_nan=True)
        return path
