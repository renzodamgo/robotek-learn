"""Telemetry of the Open v5 mission in the same CSV the Gazebo runner, judge and reports already read."""
import math
import time

from ..open_recording import OpenRecording, TELEMETRY_COLUMNS


class V5Recording(OpenRecording):
    def record_mission(self, *, sim_seconds, mission, gyro, scene, vision, odometry, command, travelled):
        if self._telemetry_file and mission.label == 'DONE' and not self._recorded_done:
            self.record_event('done', 'mission', sim_seconds)
            self._recorded_done = True
        self._telemetry_tick += 1
        if not self._telemetry_file or self._telemetry_tick % 5:
            return
        nan = math.nan
        sample = dict.fromkeys(TELEMETRY_COLUMNS, nan)
        sample.update({
            'turn_timeouts': 0, 'turn_reason': '', 'direction_reason': '', 'wall_valid': '', 'wall_yaw': scene.wall_heading,
            'limited_yaw_rate': 0.0, 'front': scene.front_axis, 'visual_age': 0.0,
            'wall_time': time.time(), 'sim_time': sim_seconds, 'state': mission.label,
            'turns': mission.completed, 'yaw_imu': mission.yaw, 'gyro': gyro,
            'steer_actual': None, 'steer_right_actual': None,
            'v_cmd': command.linear.x, 'w_cmd': command.angular.z,
            'left': scene.left, 'right': scene.right, 'front_axis': scene.front_axis,
            'turn_threshold': mission.p.turn_front_distance, 'corner_distance': scene.island_end,
            'yaw_ref': mission.goal, 'heading_error': mission.goal - mission.yaw,
            'selected_direction': mission.direction, 'visual_direction': vision.direction,
            'blue_length': vision.blue_length, 'orange_length': vision.orange_length,
            'vision_confidence': vision.confidence, 'distance_source': 'lidar', 'travelled': travelled,
            'lidar_odom_quality': odometry.last_quality, 'lidar_odom_matches': odometry.matches,
            'turn_trigger': f'arc={mission.arc} dist={mission.travelled:.2f} front={mission.front:.2f}',
        })
        self._telemetry.writerow(sample)
        self._telemetry_file.flush()
        if self._sync_seconds > 0.0 and time.monotonic() - self._last_sync >= self._sync_seconds:
            self.sync()
