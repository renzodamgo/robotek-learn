"""Fixed parallel park (``park_mode`` 1): a hard-coded manoeuvre whose every step ends on a sensor.

The planner of ``park.py`` drives ~14 short planned moves at 0.06 m/s. The real car of 8 nov 2026
(54:30 gears) does not move at that speed with full lock, and with a speed floor (~0.25 m/s) it rolls
3-12 cm after each stop, past the short moves' targets. This manoeuvre has no plan: each step uses
all the room it has and ends on the gyro heading or on the LiDAR pose against the lot, anticipating
the roll after the stop (the measured roll, as in ``park.py``).

1. APPROACH: forward along a line ``GAP`` off the lot, until base_link is ``park_fixed_ahead`` past
   the lot's far limit (LiDAR pose), aligned with the line; back along it if the line-up overshot.
2. ENTRY: reverse at full lock toward the wall until the heading is ``park_fixed_entry`` (gyro), or
   the guard (the outline, the roll ahead, closer than ``park_fixed_margin`` to a limit or the wall).
3. DOWN: reverse straight until the guard (the rear near the wall or the near limit).
4. STRAIGHTEN: alternate forward/reverse at full lock, both turning toward parallel, each until
   parallel (gyro) or the guard (``park.Parking._straighten``).
5. CENTRE: straight to the middle of the lot (LiDAR pose). Then PARKED or PARK_FAIL (it stays).

The lot (``park_lot_length`` x ``park_lot_depth`` against the outer wall, far inner face at
``park_lot_end`` along the start straight in CCW, mirrored in CW) defaults to the rules' lot and can be
set to a measured field. Frame and approach as in ``park.py``.
"""
import math

from .car import OUTLINE_HALF_WIDTH

from .field import wrap
from .parking_search import Lot, move
from .park import GUARD_AHEAD, LINE_UP, MAX_RESUME, REACH_SLACK, RUN_PAST, STRAIGHTEN_MOVES, Parking

GAP = 0.04                  # m between the car side and the lot line while approaching
ENTRY_MAX = 0.60            # m: the entry arc runs at most this far
DOWN_MAX = 0.30             # m: the straight into the lot runs at most this far
GUARD_ROLL = 1.0            # x the measured roll added to the guard's look-ahead
LADDER_MOVES = 24           # straightening moves at most (park.py: 12; 4-8 deg each from ~45 deg here)
ROLL_MAX = 0.15             # m: a longer roll (a push, a bad pose) does not enter the estimate


def make_lot(sign, p):
    """The lot of the parameters (defaults: the rules) in the v3 track frame."""
    lot = Lot(sign)
    length, depth, end = p.park_lot_length, p.park_lot_depth, p.park_lot_end
    if sign > 0:
        lot.xa, lot.xb = end - length, end
    else:
        lot.xa, lot.xb = -end, -end + length
    lot.y1 = lot.y0 + depth
    return lot


class FixedParking(Parking):
    """PARK_APPROACH -> PARK_PLAN -> (PARK_STEER -> PARK_MOVE -> PARK_SETTLE)* -> PARKED or PARK_FAIL."""

    def __init__(self, mission, speed, margin):
        super().__init__(mission, speed, margin)
        p = mission.p
        self.replan = False
        self.lot = make_lot(1.0 if mission.selected_direction == 'ccw' else -1.0, p)
        self.ahead = p.park_fixed_ahead
        self.entry = p.park_fixed_entry
        self.guard_margin = p.park_fixed_margin
        self.coast = p.park_fixed_roll
        self.ladder = 0             # straightening moves driven

    def _line(self):
        return self.lot.y1 + OUTLINE_HALF_WIDTH + GAP

    def _approach(self, now, x, y, th):
        line, target = self._line(), self.lot.xb + self.ahead
        aligned = abs(y - line) < 0.008 and abs(wrap(th)) < math.radians(2.0)
        if self.state == 'PARK_APPROACH' and x >= target and (aligned or x >= target + RUN_PAST):
            self.state = 'PARK_BACK'
        if self.state == 'PARK_BACK':
            return self._back(now, x, y, th, line, target)
        steer = 2.0 * wrap(math.atan2(line - y, 0.30) - th)
        speed = 2.0 * self.speed if target - x > 0.30 else self.speed
        return speed, max(-self.steer_limit, min(self.steer_limit, steer))

    def _plan(self, now, pose):
        if self.plans == 0:          # the approach line of this lot, not park.py's
            line = self._line()
            if abs(pose[1] - line) >= 0.015 or abs(wrap(pose[2])) >= math.radians(3.0):
                if self.reapproaches < 2:
                    self.reapproaches += 1
                    self.back_to, self.state = pose[0] - 0.50, 'PARK_BACK'
                    self.log.append(dict(event='park_reapproach', pose=list(pose)))
                    return
            self._load(pose)
        self._next(now, pose)

    def _load(self, pose):
        """The fixed steps: a straight to the start pose if the approach stopped off it, the entry
        arc and the straight into the lot. Straightening and centring follow (``_finishing``)."""
        self.plans = 1
        start = self.lot.xb + self.ahead
        if abs(pose[0] - start) > LINE_UP:
            self.moves.append((0.0, 1.0 if pose[0] < start else -1.0, abs(pose[0] - start),
                               ('along', (start, pose[1], 0.0))))
        self.moves.append((-self.steer_limit, -1.0, ENTRY_MAX, ('heading', self.entry)))
        self.moves.append((0.0, -1.0, DOWN_MAX, ('guard', None)))
        self.log.append(dict(event='park_fixed', pose=list(pose), entry=self.entry, ahead=self.ahead))

    def _straighten(self, pose):
        """``park.Parking._straighten`` with up to LADDER_MOVES moves."""
        if self.ladder < LADDER_MOVES:   # park.py stops at its own 12
            self.straightened = min(self.straightened, STRAIGHTEN_MOVES - 1)
        move_ = super()._straighten(pose)
        if move_ is not None:
            self.ladder += 1
        return move_

    def _reached(self, x, y, th, early):
        if self.current[3][0] == 'guard':
            return False             # only the guard (or the length) ends it
        return super()._reached(x, y, th, early)

    def _settled(self, now, pose):
        """As ``park.Parking._settled``, with the guard of this manoeuvre and rolls up to ROLL_MAX
        (the floor speed of the real car rolls 3-12 cm)."""
        rolled = math.hypot(pose[0] - self.stop_pose[0], pose[1] - self.stop_pose[1])
        self.coast = 0.5 * self.coast + 0.5 * min(rolled, ROLL_MAX)
        steer, direction = self.current[:2]
        ahead = move(*pose, steer, direction * (GUARD_AHEAD + GUARD_ROLL * self.coast))
        if (not self._reached(*pose, REACH_SLACK) and self.resumes < MAX_RESUME
                and self.lot.free(*ahead, steer, self.guard_margin)):
            self.resumes += 1
            self.state = 'PARK_MOVE'
            return
        self._next(now, pose)

    def _drive(self, now, x, y, th):
        steer, direction, length, _, (x0, y0, _) = self.current
        overrun = math.hypot(x - x0, y - y0) >= length
        look = GUARD_AHEAD + GUARD_ROLL * self.coast
        ahead = move(x, y, th, steer, direction * look)
        if overrun or not self.lot.free(*ahead, steer, self.guard_margin):
            self.resumes = 99        # stopped by the guard or too long: do not go on
        elif not self._reached(x, y, th, self.coast):
            return direction * self.speed, steer
        self.stop_pose, self.state, self.since = (x, y, th), 'PARK_SETTLE', now
        return 0.0, steer
