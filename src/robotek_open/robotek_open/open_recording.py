"""Write Open sensor evidence and events without owning ROS or control logic."""

import csv
import gzip
import json
import math
import os
from pathlib import Path
import time


TELEMETRY_COLUMNS = (
    'wall_time', 'sim_time', 'state', 'turns',
    'x_gt', 'y_gt', 'yaw_gt', 'x_odom',
    'y_odom', 'x_map', 'y_map', 'yaw_imu',
    'gyro', 'limited_yaw_rate', 'steer_actual', 'steer_right_actual',
    'v_cmd', 'w_cmd', 'left', 'right',
    'front', 'front_axis', 'turn_threshold', 'corner_distance',
    'wall_yaw', 'wall_valid', 'yaw_ref', 'heading_error',
    'turn_reason', 'turn_timeouts', 'selected_direction', 'direction_reason',
    'visual_direction', 'visual_age', 'blue_length', 'orange_length',
    'vision_confidence', 'distance_source', 'travelled', 'lidar_odom_quality',
    'lidar_odom_matches', 'turn_trigger',
)


class OpenRecording:
    """Own all strategy files; disabled recording uses the same call interface."""

    def __init__(self, telemetry_path, sync_seconds=0.0):
        self._sync_seconds = sync_seconds   # >0: fsync files this often (real car: power cuts)
        self._last_sync = time.monotonic()
        self._telemetry_file = None
        self._scans_file = None
        self._events_file = None
        self._camera_file = None
        self._telemetry_tick = 0
        self._recorded_done = False
        if telemetry_path:
            path = Path(telemetry_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            self._telemetry_file = path.open('w', newline='')
            self._scans_file = gzip.open(str(path) + '.scans.jsonl.gz', 'wt')
            self._events_file = (path.parent / 'events.jsonl').open('w')
            self._telemetry = csv.DictWriter(
                self._telemetry_file, fieldnames=TELEMETRY_COLUMNS)
            self._telemetry.writeheader()
            self._camera_file = (path.parent / 'camera-evidence.csv').open('w', newline='')
            self._camera_csv = csv.writer(self._camera_file)
            self._camera_csv.writerow(['stamp', 'blue_length', 'orange_length',
                                       'confidence', 'candidate', 'hits', 'direction'])

    def record_camera(self, stamp, vision):
        if self._camera_file:
            self._camera_csv.writerow([stamp, vision.blue_length,
                                       vision.orange_length, vision.confidence,
                                       vision.candidate, vision.hits,
                                       vision.direction])
            self._camera_file.flush()

    def record_scan(self, msg, sim_seconds):
        if self._scans_file:
            self._scans_file.write(json.dumps({
                'sim_time': sim_seconds,
                'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'angle_min': msg.angle_min, 'angle_increment': msg.angle_increment,
                'range_min': msg.range_min, 'range_max': msg.range_max,
                'ranges': [r if math.isfinite(r) else None for r in msg.ranges],
            }) + '\n')

    def sync(self):
        """Push telemetry, scans and events to the disk, not just to the OS cache."""
        self._last_sync = time.monotonic()
        for stream in (self._telemetry_file, self._scans_file, self._events_file, self._camera_file):
            if stream:
                stream.flush()   # GzipFile: a sync flush, the data so far stays readable
                os.fsync(stream.fileno())

    def close(self):
        for stream in (self._telemetry_file, self._scans_file,
                       self._camera_file, self._events_file):
            if stream:
                stream.close()

    def record_event(self, event, source, sim_seconds):
        if self._events_file:
            self._events_file.write(json.dumps({
                'event': event, 'source': source,
                # Real wall time (the node clock follows /clock in simulation).
                'wall_time': time.time(),
                'sim_time': sim_seconds,
            }) + '\n')
            self._events_file.flush()
            if self._sync_seconds > 0.0:
                os.fsync(self._events_file.fileno())
