"""The few constants of the Open that are not strategy parameters: where the LiDAR sits, what counts as a line in
the camera image and the control rate. Match the geometry to the URDF.
"""
from dataclasses import dataclass

from robotek_common import vehicle


@dataclass(frozen=True)
class OpenHardware:
    lidar_offset_x: float = vehicle.LIDAR_OFFSET_X


@dataclass(frozen=True)
class OpenSensors:
    vision_min_length: float = 40.0        # px: a shorter line is noise
    vision_min_confidence: float = 0.25    # how much longer one line must be than the other


HARDWARE = OpenHardware()
SENSORS = OpenSensors()
CONTROL_RATE = 50.0
