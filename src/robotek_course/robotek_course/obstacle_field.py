"""The Obstacle field by the rules (signs, parking lot, Gazebo world): see ``robotek_evaluation.obstacle_field``.

Re-exported here so the course has one place to import from; it is simulation only, and your strategy must not
use it (the car does not know where the signs are: it has to see them).
"""
from robotek_evaluation.obstacle_field import (PARKING_LOT, SPAWN_PROGRESS, build_world,  # noqa: F401
                                               regulation_signs)
