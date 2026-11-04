"""ROS parameters of the Open v5 node: the mission's own plus the node's and the camera's.

The defaults are the real car's (car.launch.py starts the node without a parameter file): start with the board
button, keep the car stopped after DONE, telemetry in ~/runs, the profile values of the Gazebo runs (open_v5.yaml
of WRO-Strategies) and the camera colours measured on the car (REAL_OPEN_CAMERA of its robot.launch.py).
"""
from dataclasses import asdict

from .mission import Params

OPERATION = {
    # 'auto' = ~/runs/<date>-<time>-open-v5/telemetry.csv; '' = no telemetry.
    'telemetry_path': 'auto', 'auto_start': False, 'shutdown_on_done': False, 'done_hold': 35.0,
    'scan_max_age': 0.35, 'imu_max_age': 0.2, 'telemetry_sync_seconds': 0.5,  # sync: a power cut lost a run
}
# Camera colours (OpenCV HSV: h_min, h_max, s_min, v_min; h_min > h_max wraps through red).
VISION = {
    'direction_confirmations': 3,
    'vision_blue_h_min': 100, 'vision_blue_h_max': 135, 'vision_blue_s_min': 80, 'vision_blue_v_min': 25,
    'vision_orange_h_min': 158, 'vision_orange_h_max': 4, 'vision_orange_s_min': 60, 'vision_orange_v_min': 40,
    'vision_floor_mask': True, 'vision_wall_v_max': 40, 'vision_wall_min_rows': 8,
}
PARAMETERS = {**asdict(Params()), **OPERATION, **VISION}


def validate_parameters(params, allowed=()):
    """Reject names the v5 node does not know (typos)."""
    unknown = set(params) - set(PARAMETERS) - set(allowed)
    if unknown:
        raise ValueError('Unsupported Open v5 parameters: ' + ', '.join(sorted(unknown))
                         + '. The parameters and their defaults are in robotek_open/v5/settings.py.')
