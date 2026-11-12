"""Step 4 - The lane: stay at a fixed distance from the island wall."""
from robotek_course.minisim import clamp
from .heading import heading_steer

GAP = 0.25            # m kept from the island wall
GAIN = 0.3            # rad of heading offset per metre of gap error
LIMIT = 0.4           # rad: the offset never asks for more than this


def lateral_bias(inner, sign=1, gap=GAP):
    """Heading offset that moves the car to ``gap`` from the island wall.

    ``inner`` is the measured distance to the island wall; ``sign`` is +1 when the island is on the left (counter-
    clockwise lap) and -1 when it is on the right. Farther than ``gap``: aim a little towards the island.
    """
    return clamp(sign * GAIN * (inner - gap), -LIMIT, LIMIT)


def lane_steer(goal, yaw, inner, sign=1):
    """The heading law of step 2, aimed at the goal heading plus the lateral offset."""
    return heading_steer(goal + lateral_bias(inner, sign), yaw)
