"""The line (step 4): one side offset per sign, held around it, straight ramps in between.

Red is passed on the right and green on the left. In the track frame the lane normal points to
the island (left), so a sign at lateral ``row`` (+0.1 island row, -0.1 outer row) is passed with
the car's centre at ``row - clearance`` (red) or ``row + clearance`` (green), where clearance is
the gap asked for plus half the car's width. The plan is a list of knots (progress, offset):
the offset is held from ``HOLD`` before a sign to ``HOLD`` after it, and the line between two
signs is a straight ramp, so the move to the next side starts right after the previous sign.

A sign whose colour is still unknown is given the side away from it (``UNKNOWN_OFFSET``), and a
sign of unknown colour at the start of the next straight gets a *lookout*: the line passes the
island side 1.5 units before it, where the camera can see it.
"""
from .lane import UNITS_PER_LAP, clamp

UNKNOWN_OFFSET = 0.12   # m away from a sign whose colour is still unknown
LOOKOUT = 0.15          # m to the island side to see an inner sign ahead
HOLD = 0.15             # track units before and after a sign the side offset is held
HORIZON = 3.0           # track units of remembered signs the plan covers
OFFSET_LIMIT = 0.36     # m from the lane centre (the car is 0.148 m wide, the lane 1 m)
LOT_OFFSET = -0.19      # m; beside the parking lot the car keeps off its limits


def side_offset(lateral, colour, clearance):
    """Lateral offset (m, + island side) that passes a sign on its side; away from it if unknown."""
    if colour is None:
        # Sign rows are 0.1 m off the centre and the car is 0.148 m wide.
        return -UNKNOWN_OFFSET if lateral >= 0.0 else UNKNOWN_OFFSET
    offset = lateral - clearance if colour == 'red' else lateral + clearance
    return clamp(offset, -OFFSET_LIMIT, OFFSET_LIMIT)


def with_offsets(signs, clearance):
    """{progress: (lateral, colour)} -> {progress: (lateral, colour, offset)}."""
    return {s: (lateral, colour, side_offset(lateral, colour, clearance)) for s, (lateral, colour) in signs.items()}


def lookouts(signs, phi):
    """Knots that bring a corner-exit sign of unknown colour into the camera's view.

    The camera sees a sign at the start of the next straight from the middle of the current one
    only from the island side of the lane (+0.15 m; 0 m for the outer row), so the line passes
    there 1.5 track units before the sign when no other sign is near.
    """
    out = []
    for s, (lateral, colour, _) in signs.items():
        look = s - 1.5
        if colour is not None or round(s) % 2 or look - phi < 0.1:
            continue
        if any(abs(other - look) < 0.4 for other in signs):
            continue
        out.append((look, LOOKOUT if lateral > 0.0 else 0.0))
    return out


def sign_knots(signs, phi):
    """The knots of the signs (held offsets) and of the lookouts, not sorted."""
    knots = [(s + d, sign[2]) for s, sign in signs.items() for d in (-HOLD, HOLD)]
    return knots + lookouts(signs, phi)


def offset_at(knots, progress):
    """Planned offset at a progress value: linear between sorted knots, constant past the ends
    (0 without knots)."""
    if not knots:
        return 0.0
    if progress <= knots[0][0]:
        return knots[0][1]
    for (s0, d0), (s1, d1) in zip(knots, knots[1:]):
        if progress <= s1:
            return d0 + (d1 - d0) * (progress - s0) / max(s1 - s0, 1e-9)
    return knots[-1][1]


def lot_zone(progress, direction):
    """Whether a progress value is on the start straight beside the parking lot (track frame:
    x 0.13..0.48 in CCW, -0.48..-0.13 in CW, plus the car's length)."""
    s = progress % UNITS_PER_LAP
    low, high = (0.43, 1.18) if direction == 'ccw' else (-0.18, 0.57)
    return low <= s <= high or low <= s - UNITS_PER_LAP <= high
