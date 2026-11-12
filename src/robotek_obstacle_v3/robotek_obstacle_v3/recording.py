"""What an Obstacle v3 round on the car leaves behind, as the Open v5 does (12 nov 2026).

One folder per launch, ``~/runs/<date>-<time>-obstacle-v3/``:

* ``telemetry.csv``: one row every ``every`` control ticks (25 Hz by default): time since Start, state, progress
  and lap, pose, gyro, commands, the nearest LiDAR ranges ahead / left / right, the rear sensor, the next sign, the
  parking's sub-state and the age of each sensor;
* ``telemetry.csv.scans.jsonl.gz``: every LiDAR scan as it arrived;
* ``signs.csv``: every camera observation paired with the LiDAR (stamp, colour, x / y in the car frame, bearing,
  range, camera-LiDAR angular error);
* ``events.jsonl``: start, each corner and lap, recover, lot exit, parking verdict, WAIT_SENSOR, done / fault, stop;
* ``params.yaml``: the parameters the strategy ran with (the ones changed by hand and the full set).

The files are flushed every row and fsynced every ``sync_seconds`` (the car can lose power mid-round)."""
import csv
import gzip
import json
import math
import os
from pathlib import Path
import time

COLUMNS = (
    'wall_time', 't', 'state', 'progress', 'lap', 'corner', 'x', 'y', 'th', 'yaw', 'gyro', 'gyro_bias', 'travelled',
    'v_cmd', 'steer_cmd', 'front', 'left', 'right', 'rear_mm', 'pillar_lateral', 'pillar_ahead', 'pillar_colour',
    'offset', 'park_state', 'scan_age', 'imu_age', 'camera_age', 'signs_seen')


def _clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, (list, tuple)):
        return [_clean(v) for v in value]
    if isinstance(value, dict):
        return {k: _clean(v) for k, v in value.items()}
    return value


def _plain(value):
    """numpy values and anything else json does not know."""
    return value.tolist() if hasattr(value, 'tolist') else str(value)


def sector_min(scan, center, half=math.radians(10.0)):
    """Smallest valid range within +-half of a bearing (car frame), whatever the message's angle layout (the car's
    LD19 gives 0..2 pi, Gazebo -110..+110 deg). inf without one."""
    if scan is None:
        return math.inf
    best = math.inf
    for k, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < scan.range_min:
            continue
        a = scan.angle_min + k * scan.angle_increment - center
        if abs(math.atan2(math.sin(a), math.cos(a))) <= half and r < best:
            best = r
    return best


def default_folder():
    return os.path.join(os.path.expanduser('~/runs'), time.strftime('%Y%m%d-%H%M%S') + '-obstacle-v3')


class Recording:
    def __init__(self, folder, every=2, sync_seconds=1.0):
        self.folder = folder
        self.every = max(1, int(every))
        self.sync_seconds = sync_seconds
        self._tick = 0
        self._last_sync = time.monotonic()
        self._files = []
        if not folder:
            return
        path = Path(folder)
        path.mkdir(parents=True, exist_ok=True)
        self._telemetry_file = self._open(path / 'telemetry.csv', newline='')
        self._telemetry = csv.DictWriter(self._telemetry_file, fieldnames=COLUMNS)
        self._telemetry.writeheader()
        self._scans = gzip.open(str(path / 'telemetry.csv.scans.jsonl.gz'), 'wt')
        self._files.append(self._scans)
        self._events = self._open(path / 'events.jsonl')
        self._signs_file = self._open(path / 'signs.csv', newline='')
        self._signs = csv.writer(self._signs_file)
        self._signs.writerow(['stamp', 'colour', 'x', 'y', 'bearing', 'range', 'angular_error'])

    def _open(self, path, **kwargs):
        stream = open(path, 'w', **kwargs)
        self._files.append(stream)
        return stream

    @property
    def on(self):
        return bool(self._files)

    def params(self, changed, full):
        if not self.on:
            return
        with open(os.path.join(self.folder, 'params.yaml'), 'w') as out:
            out.write('# Obstacle v3 on the car: parameters changed by hand, then the full set\n')
            out.write('changed:\n' + ''.join('  %s: %r\n' % kv for kv in sorted(changed.items())))
            out.write('full:\n' + ''.join('  %s: %r\n' % kv for kv in sorted(full.items())))

    def event(self, name, t=None, **data):
        if self.on:
            self._events.write(json.dumps(_clean({'event': name, 'wall_time': time.time(), 't': t, **data}),
                                         default=_plain) + '\n')
            self._events.flush()

    def scan(self, msg):
        if self.on:
            self._scans.write(json.dumps({
                'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, 'wall_time': time.time(),
                'angle_min': msg.angle_min, 'angle_increment': msg.angle_increment,
                'range_min': msg.range_min, 'range_max': msg.range_max,
                'ranges': [r if math.isfinite(r) else None for r in msg.ranges]}) + '\n')

    def signs(self, stamp, observations):
        """Each observation of the frame (associate(): a LiDAR cluster with the block's colour), even none."""
        if self.on:
            for obs in observations:
                x, y = float(obs.get('x', math.nan)), float(obs.get('y', math.nan))
                self._signs.writerow([stamp, obs.get('color'), round(x, 4), round(y, 4),
                                      round(math.atan2(y, x), 4), round(math.hypot(x, y), 4),
                                      round(float(obs.get('angular_error', math.nan)), 4)])
            self._signs_file.flush()

    def row(self, **values):
        self._tick += 1
        if not self.on or self._tick % self.every:
            return
        self._telemetry.writerow({k: values.get(k) for k in COLUMNS})
        self._telemetry_file.flush()
        if self.sync_seconds > 0.0 and time.monotonic() - self._last_sync >= self.sync_seconds:
            self.sync()

    def sync(self):
        self._last_sync = time.monotonic()
        for stream in self._files:
            stream.flush()
            os.fsync(stream.fileno())

    def close(self):
        for stream in self._files:
            try:
                stream.close()
            except (OSError, ValueError):
                pass
        self._files = []
