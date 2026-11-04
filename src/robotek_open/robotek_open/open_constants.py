"""Hardware and sensor calibration for the sensor-only Open controller.

Match hardware geometry to the URDF; tune calibration here when measuring the
real car. These values are recorded with each run and are not strategy switches.
"""
from dataclasses import dataclass

from robotek_common import vehicle


@dataclass(frozen=True)
class OpenHardware:
    wheelbase: float = vehicle.WHEELBASE
    steer_limit: float = vehicle.STEER_LIMIT
    lidar_offset_x: float = vehicle.LIDAR_OFFSET_X
    gyro_sign: float = 1.0


@dataclass(frozen=True)
class OpenSensors:
    vision_min_length: float = 40.0
    vision_min_confidence: float = 0.25
    vision_confirmations: int = 5
    side_center: float = 1.5708
    side_half_width: float = 0.44
    front_half_width: float = 0.35
    wall_angle_max: float = 0.45


HARDWARE = OpenHardware()
SENSORS = OpenSensors()
LAPS = 3
CONTROL_RATE = 50.0
DISTANCE_MODEL_ACCEL = vehicle.DISTANCE_MODEL_ACCEL
