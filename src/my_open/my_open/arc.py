"""Step 5 - The corner: turn 90 degrees by moving the reference heading in a ramp, with a feed-forward and damping."""
import math

from robotek_course.minisim import STEER_LIMIT, clamp, wrap
from .heading import KP


def ramp(ref, goal, rate, dt):
    """Move the reference heading ``ref`` towards ``goal`` by at most ``rate * dt`` radians (the short way round)."""
    gap = wrap(goal - ref)
    if abs(gap) <= rate * dt:
        return goal
    return wrap(ref + math.copysign(rate * dt, gap))


DAMPING = 0.1        # steering (rad) per rad/s of yaw-rate error
EXIT_ANGLE = 0.25    # rad: the feed-forward fades out over the last quarter radian


def arc_steer(goal, ref, yaw, gyro, rate, dt, sign=1, feed=True, fade=True, damping=DAMPING):
    """One tick of a corner: the ramp plus two terms. ``sign`` is +1 for a left turn, -1 for a right one.

    * feed-forward: while the reference is still climbing, add full lock (``sign * STEER_LIMIT``) and expect the yaw
      rate ``sign * rate``; with ``fade`` both shrink to zero over the last ``EXIT_ANGLE`` before the goal;
    * damping: subtract ``damping`` times how far the measured yaw rate is from the expected one.
    """
    ramping = ref != goal
    ref = ramp(ref, goal, rate, dt)
    scale = min(1.0, max(0.0, sign * wrap(goal - yaw)) / EXIT_ANGLE) if fade else 1.0
    steer_ff, rate_ff = (sign * STEER_LIMIT * scale, sign * rate * scale) if (ramping and feed) else (0.0, 0.0)
    steer = KP * wrap(ref - yaw) + steer_ff - damping * (gyro - rate_ff)
    return clamp(steer, -STEER_LIMIT, STEER_LIMIT), ref
