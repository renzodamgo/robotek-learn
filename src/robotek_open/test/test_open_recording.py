"""Run files must survive a power cut: fsync makes the data so far readable."""

import csv
import gzip
from types import SimpleNamespace

from robotek_open.open_recording import OpenRecording


def test_sync_leaves_readable_telemetry_scans_and_events(tmp_path):
    recording = OpenRecording(str(tmp_path / 'telemetry.csv'), sync_seconds=0.5)
    recording.record_event('start', 'start_topic', 0.0)
    stamp = SimpleNamespace(sec=1, nanosec=0)
    recording.record_scan(SimpleNamespace(header=SimpleNamespace(stamp=stamp), angle_min=0.0,
                                          angle_increment=0.1, range_min=0.1, range_max=12.0,
                                          ranges=[1.0, float('inf')]), 0.0)
    recording.sync()
    # Read while the files are still open, as after a power cut.
    assert 'start' in (tmp_path / 'events.jsonl').read_text()
    assert list(csv.reader(open(tmp_path / 'telemetry.csv')))[0][0] == 'wall_time'
    with gzip.open(tmp_path / 'telemetry.csv.scans.jsonl.gz', 'rt') as scans:
        try:
            first = scans.readline()
        except EOFError:   # no end-of-stream marker yet; the synced data was already returned
            first = ''
    assert '"ranges": [1.0, null]' in first
    recording.close()
