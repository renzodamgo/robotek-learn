"""The referee's assistant of an Obstacle round: the Open one (``round_recorder``) with the Obstacle field and judge.

* waits until the sensors and your node are up, publishes ``/start`` and writes the Start event;
* records the true pose of the car (``audit_pose.csv``; your strategy never sees it);
* ends the round after 180 s, after 30 s without reaching a new section during the laps, or when the car has
  driven the three laps (and, with the lot, parked) and stays still for 31 s, and then scores it
  (``score_obstacle``). It does not read your node's state (STOP): only the true pose. A car that stops within
  the 180 s is watched past them until its stop has lasted 31 s (the judge needs 30 s of stop observed).

The field (``world.sdf`` and ``manifest.json`` with the signs and the lot) is written by the launch before the
simulator starts; this node only adds to the manifest who drove and when.
"""
import datetime
import json

import rclpy

from .round_recorder import MIN_EXITS_TO_FINISH, REST_S, STALL_S, RoundRecorder
from .score_obstacle import format_report, score_directory


class ObstacleRoundRecorder(RoundRecorder):
    def __init__(self):
        super().__init__()
        self.get_logger().info('Obstacle round: %s' % self.directory)

    def write_manifest(self):
        path = self.directory / 'manifest.json'
        manifest = json.loads(path.read_text()) if path.exists() else {}
        manifest.update(started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                        strategy=self.declare_parameter('strategy', '').value,
                        judge='robotek_course.obstacle_round_recorder')
        self.declare_parameter('world', '')
        path.write_text(json.dumps(manifest, indent=2) + '\n')

    def supervise(self):
        """As the Open referee, except at the 180 s limit: a car that already stopped after the laps (parked, say)
        within the 3 minutes is watched until its stop has been observed for 30 s, so the stop can be judged."""
        if self.finished or self.pose is None or self.start is None:
            return super().supervise()
        now = self.now()
        self.track_progress(now)
        stopped = self.exits >= MIN_EXITS_TO_FINISH and self.rest_since - self.start < self.limit
        if stopped and now - self.rest_since >= REST_S:
            self.end('stopped_after_three_laps')
        elif now - self.start >= self.limit and not stopped:
            self.end('round_time_limit_180s')
        elif self.exits < MIN_EXITS_TO_FINISH and now - self.section_since >= STALL_S:
            self.end('no_section_progress_%ds' % STALL_S)

    def end(self, reason):
        self.finished = True
        self.event('end', reason=reason)
        self.file.close()
        self.events.close()
        print('\nRonda terminada (%s).\n' % reason, flush=True)
        try:
            print(format_report(self.directory, score_directory(self.directory)), flush=True)
        except Exception as error:   # the files stay on disk: the score can be run again by hand
            print('No se pudo puntuar la ronda: %r\nPuedes repetirlo con: ros2 run robotek_course score_obstacle %s'
                  % (error, self.directory), flush=True)
        rclpy.shutdown()


def main():
    rclpy.init()
    node = ObstacleRoundRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        if not node.finished:
            node.file.close()
            node.events.close()


if __name__ == '__main__':
    main()
