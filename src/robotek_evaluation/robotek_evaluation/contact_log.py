"""Contact log: one line per contact episode instead of one per physics step.

Gazebo publishes a contact message every physics step (1 kHz) while two bodies touch, so
a car pressed against a wall wrote ~300 KB per simulated second (238 MB in one round,
9 Oct 2026). An episode is a run of messages from the same source between the same robot
part and the same object, with gaps of at most ``EPISODE_GAP`` simulated seconds.

Each line keeps the fields of the episode's first message (``sim_time`` is when it
started, so first-contact times are unchanged) plus ``episode``, ``end_sim_time``,
``messages`` and the maxima of ``max_depth``/``points``. An episode is written when it
starts (a killed recorder never loses a contact), again every ``CHECKPOINT`` seconds while
it lasts and when it ends; readers keep the last line of each episode. Old logs with one
line per message (no ``episode`` field) are read as episodes too.
"""
import json
from pathlib import Path

EPISODE_GAP = 0.05   # s sim without a message that ends an episode
CHECKPOINT = 1.0     # s sim between rewrites of a long episode


def _key(contact):
    return (contact.get('source'), contact.get('other'), contact.get('pillar'), contact.get('robot'))


def _max(a, b):
    return b if a is None else a if b is None else max(a, b)


class EpisodeLog:
    """Group contact messages (dicts with ``sim_time``) into episodes."""

    def __init__(self, stream=None):
        self.stream = stream
        self.open = {}
        self.episodes = []
        self._written = {}

    def add(self, contact):
        key, t = _key(contact), contact['sim_time']
        episode = self.open.get(key)
        if episode is not None and t - episode['end_sim_time'] <= EPISODE_GAP:
            episode.update(end_sim_time=t, messages=episode['messages']+1,
                           max_depth=_max(episode.get('max_depth'), contact.get('max_depth')),
                           points=_max(episode.get('points'), contact.get('points')))
            if t - self._written[episode['episode']] >= CHECKPOINT:
                self._write(episode)
            return
        if episode is not None:
            self._write(episode)
        episode = dict(contact, episode=len(self.episodes), end_sim_time=t, messages=1)
        self.open[key] = episode
        self.episodes.append(episode)
        self._write(episode)

    def close(self):
        for episode in self.open.values():
            if self._written.get(episode['episode']) != episode['end_sim_time']:
                self._write(episode)
        self.open = {}

    def _write(self, episode):
        self._written[episode['episode']] = episode['end_sim_time']
        if self.stream is not None:
            self.stream.write(json.dumps(episode)+'\n')


def compact(contacts):
    """Episodes from per-message contact dicts (old log format), in start order."""
    log = EpisodeLog()
    for contact in contacts:
        log.add(contact)
    return log.episodes


def read(path):
    """Episodes from a contacts.jsonl in either format, sorted by start time."""
    records = [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
    legacy = [r for r in records if 'episode' not in r]
    latest = {}
    for r in records:
        if 'episode' in r:
            latest[r['episode']] = r
    episodes = list(latest.values()) + compact(legacy)
    return sorted(episodes, key=lambda e: e['sim_time'])


def message_count(episodes):
    """Contact messages behind a list of episodes (one per line in old logs)."""
    return sum(e.get('messages', 1) for e in episodes)
