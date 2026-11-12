"""Step 5 - The corner: full lock until the heading is close to the new goal, then a damped heading law finishes it."""
import math

from robotek_course.minisim import STEER_LIMIT, clamp, wrap
from .heading import KP

TURN_TOLERANCE = 0.3     # rad (17 deg): the full-lock part ends this short of the goal heading
DAMPING = 0.1            # steering (rad) taken off per rad/s the car is turning


def damped_steer(goal, yaw, gyro, damping=DAMPING):
    """The heading law of step 2 with a brake on the turning speed: the faster the car turns, the less it steers."""
    return clamp(KP * wrap(goal - yaw) - damping * gyro, -STEER_LIMIT, STEER_LIMIT)


def arc_steer(goal, yaw, gyro, turn_tolerance=TURN_TOLERANCE, damping=DAMPING):
    """One tick of a corner: returns (steering angle, whether the corner is done).

    Farther than ``turn_tolerance`` from the goal heading: the wheels at full lock towards it. Closer: the corner is done
    and the damped heading law takes the car the rest of the way.
    """
    error = wrap(goal - yaw)
    if abs(error) > turn_tolerance:
        return math.copysign(STEER_LIMIT, error), False
    return damped_steer(goal, yaw, gyro, damping), True
