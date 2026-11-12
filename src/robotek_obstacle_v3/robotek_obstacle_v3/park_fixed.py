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

``park_fixed_sense`` 1: the lot is placed from what the LiDAR sees, not from the map pose alone. Every
scan, the outer wall beside the lot (a line fitted to its returns) and the limit plates (returns near
their expected x, inside the lot's depth) give a correction of the pose (along-track shift, distance
to the wall, heading against the wall), low-passed and bounded; with nothing seen it keeps the last
one. On the car (8 nov 2026) the outer wall showed at y -1.52..-1.57 instead of -1.50 and a limit
~3 cm off: the map pose places the lot wrong by that much, in a lot with 2.6 cm of room per side.
"""
import math

import numpy as np

from .car import LIDAR_OFFSET_X, OUTLINE_HALF_WIDTH

from .field import wrap
from .parking_search import LIMIT_THICKNESS, Lot, move
from .park import GUARD_AHEAD, GYRO_ARC, LINE_UP, MAX_RESUME, REACH_SLACK, RUN_PAST, STRAIGHTEN_MOVES, Parking

GAP = 0.04                  # m between the car side and the lot line while approaching
ENTRY_MAX = 0.60            # m: the entry arc runs at most this far
DOWN_MAX = 0.30             # m: the straight into the lot runs at most this far
GUARD_ROLL = 1.0            # x the measured roll added to the guard's look-ahead
LADDER_MOVES = 24           # straightening moves at most (park.py: 12; 4-8 deg each from ~45 deg here)
ROLL_MAX = 0.30             # m: a longer roll (a push, a bad pose) does not enter the estimate
# park_fixed_sense: the lot from the LiDAR.
SENSE_GAIN = 0.3            # of the measured error taken per scan
SENSE_LIMIT = (0.12, 0.12, math.radians(8.0))   # largest correction: along, across (m), heading (rad)
WALL_BAND = 0.10            # m around the expected outer wall for its returns
WALL_REACH = 0.6            # m beyond the lot's ends along x where the wall is fitted
WALL_MIN = 15               # returns to fit the wall; its slope only over at least WALL_SPAN m
WALL_SPAN = 0.15
PLATE_BAND = 0.06           # m around a limit plate's expected x for its returns
PLATE_MIN = 3
SENSE_DEADBAND = (0.008, 0.008, math.radians(0.75))   # smaller parts of the correction are not applied


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
        # park_fixed_roll_arc > 0: the roll after a stop is measured apart for arcs and straights (8 nov floor test at
        # ~0.3 m/s: 0.13 m straight, ~0.07 m of arc at full lock); self.coast is the one of the current move. 0: one
        # estimate for both (at 0.06 m/s it parks better: lab real_v3 195/200 against 164 with the split).
        self.split = p.park_fixed_roll_arc > 0.0
        self.rolls = {False: p.park_fixed_roll, True: p.park_fixed_roll_arc if self.split else p.park_fixed_roll}
        self.coast = p.park_fixed_roll
        self.ladder = 0             # straightening moves driven
        self.sense = bool(p.park_fixed_sense)
        self.fix = [0.0, 0.0, 0.0]  # correction of the map pose: along x, across y (m), heading (rad)
        self.pivot = (0.5 * (self.lot.xa + self.lot.xb), self.lot.y0)   # the heading turns about the lot
        self.raw = None             # last map pose
        self.seen = [0, 0]          # scans that saw the wall, a limit plate
        # park_fixed_partial 1 (12 nov 2026): a basic partial park, only APPROACH, ENTRY and DOWN (the rear into the
        # lot at ~park_fixed_entry), no straightening or centring: the car ends partly in the lot and stays.
        self.partial = bool(p.park_fixed_partial)

    # ------------------------------------------------------- the lot from the LiDAR
    def _applied(self):
        """The correction in use: each part only once it is larger than SENSE_DEADBAND. Smaller corrections mostly
        add their noise (lab real_v3, rules field: 157/200 with every correction, 188 with the deadband, 195 off)."""
        return [v if abs(v) >= d else 0.0 for v, d in zip(self.fix, SENSE_DEADBAND)]

    def _corrected(self, x, y, th, fix=None):
        dx, dy, dth = self._applied() if fix is None else fix
        cx, cy = self.pivot
        c, s = math.cos(dth), math.sin(dth)
        return cx + c * (x - cx) - s * (y - cy) + dx, cy + s * (x - cx) + c * (y - cy) + dy, wrap(th + dth)

    def step(self, now, x, y, th):
        self.raw = (x, y, th)
        return super().step(now, *self._corrected(x, y, th))

    def observe(self, bx, by):
        """A new scan (base_link points, mirrored in CW): correct the pose against the outer wall and the
        limit plates beside the lot (park_fixed_sense)."""
        if not self.sense or self.raw is None:
            return
        x, y, th = self._corrected(*self.raw, self.fix)    # the features against the full correction
        c, s = math.cos(th), math.sin(th)
        px = x + c * np.asarray(bx) - s * np.asarray(by)
        py = y + s * np.asarray(bx) + c * np.asarray(by)
        lot, (cx, _) = self.lot, self.pivot
        plates = (lot.xa - 0.5 * LIMIT_THICKNESS, lot.xb + 0.5 * LIMIT_THICKNESS)
        off_plates = np.all([np.abs(px - p) > PLATE_BAND for p in plates], axis=0)
        wall = ((np.abs(py - lot.y0) < WALL_BAND) & (px > lot.xa - WALL_REACH) & (px < lot.xb + WALL_REACH)
                & off_plates)
        g = SENSE_GAIN
        if wall.sum() >= WALL_MIN:
            wx, wy = px[wall], py[wall]
            if wx.max() - wx.min() >= WALL_SPAN:
                slope, icpt = np.polyfit(wx, wy, 1)
            else:
                slope, icpt = 0.0, float(np.median(wy))
            self.fix[1] -= g * (icpt + slope * cx - lot.y0)
            self.fix[2] -= g * math.atan(slope)
            self.seen[0] += 1
        errors = []
        lidar = x + LIDAR_OFFSET_X * c
        for p in plates:
            # The LiDAR sees the plate's face toward it (the plates are LIMIT_THICKNESS thick).
            low, high = p - 0.5 * LIMIT_THICKNESS, p + 0.5 * LIMIT_THICKNESS
            face = low if lidar < low else high if lidar > high else p
            near = (py > lot.y0 + 0.03) & (py < lot.y1 - 0.01) & (np.abs(px - p) < PLATE_BAND)
            if near.sum() >= PLATE_MIN:
                errors.append(float(np.median(px[near])) - face)
        if errors:
            self.fix[0] -= g * sum(errors) / len(errors)
            self.seen[1] += 1
        self.fix = [max(-m, min(m, v)) for v, m in zip(self.fix, SENSE_LIMIT)]

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
        self.log.append(dict(event='park_fixed', pose=list(pose), entry=self.entry, ahead=self.ahead,
                             fix=[round(v, 4) for v in self.fix], seen=list(self.seen)))

    def _finishing(self, pose):
        if self.partial:
            return None              # partial: end after the entry and the straight into the lot
        return super()._finishing(pose)

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
        steer, direction = self.current[:2]
        kind = self._is_arc(steer) and self.split
        self.rolls[kind] = 0.5 * self.rolls[kind] + 0.5 * min(rolled, ROLL_MAX)
        self.coast = self.rolls[kind]
        ahead = move(*pose, steer, direction * (GUARD_AHEAD + GUARD_ROLL * self.coast))
        if (not self._reached(*pose, REACH_SLACK) and self.resumes < MAX_RESUME
                and self.lot.free(*ahead, steer, self.guard_margin)):
            self.resumes += 1
            self.state = 'PARK_MOVE'
            return
        self._next(now, pose)

    def _is_arc(self, steer):
        return abs(steer) >= GYRO_ARC * self.steer_limit

    def _next(self, now, pose):
        super()._next(now, pose)
        if self.state in ('PARKED', 'PARK_FAIL') and self.log and self.log[-1].get('event') == 'park_end':
            self.log[-1].update(fix=[round(v, 4) for v in self.fix], seen=list(self.seen), partial=self.partial)
        if self.current is not None:
            self.coast = self.rolls[self._is_arc(self.current[0]) and self.split]

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
