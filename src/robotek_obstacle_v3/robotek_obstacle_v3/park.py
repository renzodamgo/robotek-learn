"""Parallel parking after the three laps: a fixed manoeuvre closed on the gyro and the LiDAR.

The lot is 1.5 car lengths (0.35 m) by 0.20 m against the outer wall of the start straight; with
the measured car (0.233 m) a single reverse manoeuvre would need ~0.42 m, so parking takes
~14 short moves. The car may stop as long as it needs (only the 3 minutes count), so:

1. APPROACH: forward along a line 4 cm off the lot, to a pose 5 cm past its far limit.
2. The nominal plan (a weighted A* over 3 cm full/half-lock arcs, ``obstacle_parking.search``,
   with the measured outline and the steered wheels) is computed ONCE before the round, from that
   pose: the lot and the approach line are fixed by the rules. No search runs during the round.
3. STEER / MOVE: every planned move ends at an absolute target of the plan, not after a length,
   so errors do not add up: an arc at the planned heading (gyro), a steep straight at the planned
   depth (LiDAR: distance to the outer wall), another straight at the planned point along its
   heading. The guard (the outline 2 cm ahead closer than 12 mm to a limit or the wall) ends a
   move early; the next move goes on from there.
4. STRAIGHTEN: alternate forward/reverse at full lock, both turning toward parallel, each until
   parallel or the guard. CENTRE: straight to the middle of the lot. Then PARKED (inside and
   parallel, own estimate) or PARK_FAIL (it stays where it is: partly parked still scores 7).

Frame: the v3 track frame (start straight at the bottom, travel +x, outer wall at y = -1.5; CW
rounds mirrored). The lot is at x in [0.48 - L, 0.48] in CCW and [-0.48, -0.48 + L] in CW; the
geometry seen from the approach line is the same in both, only shifted along x.
"""
import math
import time
from functools import lru_cache

from .car import OUTLINE_FRONT, OUTLINE_HALF_WIDTH, OUTLINE_REAR, WHEELBASE

from .field import wrap
from .parking_search import STEP, SUBSTEPS, Lot, move, search

GAP = 0.04                  # m between the car side and the lot line while approaching
RUN_PAST = 0.40             # m past the planning pose the car may drive to line up
AHEAD = 0.05                # m from the lot's far limit to base_link where the car stops to plan
PLAN_MARGINS = (0.015, 0.01)  # m kept from the limits and the wall by the plan (first that works)
GUARD_MARGIN = 0.012        # m; the outline 2 cm ahead closer than this ends a move
GUARD_AHEAD = 0.02
DEPTH_HEADING = math.radians(20.0)  # straights steeper than this end at the planned depth
OVERRUN = 0.10              # m a planned move may run past its length looking for its target
STRAIGHTEN = math.radians(1.0)      # |heading| that ends the straightening
STRAIGHTEN_MOVES = 12
STUCK = 0.005               # m; two straightening moves shorter than this end the straightening
LINE_UP = 0.005             # m off the planning pose along x that a first straight move corrects
ALIGNED = (0.015, math.radians(3.0))  # m off the line and heading to start the manoeuvre
REAPPROACH = 0.50           # m backed along the line before approaching again when not aligned
MAX_REAPPROACH = 2
REACH_SLACK = 0.003         # m of a target a stopped move may miss; it goes on if it missed more
MAX_RESUME = 2
STEER_CREEP = 0.003         # m/s while the servo turns: above the 1 mm/s that carries the steering
                            # angle in the Twist (it is kept at v = 0), below the 5 mm/s motor deadband
GYRO_ARC = 0.45             # of the steering limit: arcs at least this sharp measure distance by gyro
STEER_TIME = (0.2, 1.0 / 1.2)  # s stopped for the servo: delay + its 1.2 rad/s sweep to the next angle
COAST = 0.012              # m the car rolls after a stop command at first; then the measured roll
SETTLE_TIME = 0.3           # s stopped before checking the result
PARALLEL = math.radians(3.0)
INSIDE = 0.005              # m inside every side of the lot to call it parked


@lru_cache(maxsize=4)
def nominal_plan(steer_limit):
    """Arcs from the planning pose, computed once before the round (the lot and the approach
    line are fixed by the rules; CW is the same lot shifted along x). A few seconds; the
    corrections planned during the round start close to the lot and take milliseconds."""
    lot = Lot(1.0)
    pose = (lot.xb + AHEAD, lot.y1 + OUTLINE_HALF_WIDTH + GAP, 0.0)
    for margin in PLAN_MARGINS:
        path, _ = search(lot, pose, None, steer_limit, margin, time.monotonic() + 60.0, 2.0)
        if path:
            return tuple(path)
    return ()


def _moves(path):
    """Plan steps merged into moves: (steer, direction, length)."""
    moves = []
    for steer, direction in path:
        if moves and moves[-1][:2] == (steer, direction):
            moves[-1] = (steer, direction, moves[-1][2] + STEP)
        else:
            moves.append((steer, direction, STEP))
    return moves


def _target(steer, end):
    """What ends a planned move: the end heading of an arc, the depth of a steep straight, or the
    end point along a straight's heading."""
    x, y, th = end
    if abs(steer) > 1e-6:
        return ('heading', th)
    if abs(wrap(th)) > DEPTH_HEADING:
        return ('depth', y)
    return ('along', (x, y, th))


@lru_cache(maxsize=4)
def nominal_moves(steer_limit):
    """Moves of the nominal plan with their targets in the CCW frame:
    ((steer, direction, length, target), ...); empty without a plan."""
    lot = Lot(1.0)
    pose = (lot.xb + AHEAD, lot.y1 + OUTLINE_HALF_WIDTH + GAP, 0.0)
    out = []
    for steer, direction, length in _moves(nominal_plan(steer_limit)):
        steps = round(SUBSTEPS * length / STEP)
        for _ in range(steps):           # the same integration as the search's arcs
            pose = move(*pose, steer, direction * length / steps)
        out.append((steer, direction, length, _target(steer, pose)))
    return tuple(out)


class Parking:
    """PARK_APPROACH -> PARK_PLAN -> (PARK_STEER -> PARK_MOVE)* -> PARKED or PARK_FAIL."""

    def __init__(self, mission, speed, margin):
        self.speed = speed
        self.steer_limit = mission.p.steer_limit
        self.lot = Lot(1.0 if mission.selected_direction == 'ccw' else -1.0)
        self.state = 'PARK_APPROACH'
        self.moves = []             # [(steer, direction, length, target)]
        self.current = None         # (steer, direction, length, target, start pose)
        self.since = None
        self.plans = 0              # 1 once the nominal plan is loaded (no search during the round)
        self.driven = 0             # moves driven
        self.straightened = 0
        self.short = 0              # straightening moves in a row shorter than STUCK
        self.centred = False
        self.coast = COAST          # m rolled after a stop command (running estimate)
        self.stop_pose = None       # pose when the last move was told to stop
        self.resumes = 0            # times the current move went on after stopping short
        self.back_to = None         # x to back up to before approaching again (not aligned)
        self.reapproaches = 0
        self.log = []

    def step(self, now, x, y, th):
        """-> (speed, steer) in the track frame."""
        if self.state in ('PARK_APPROACH', 'PARK_BACK'):
            return self._approach(now, x, y, th)
        if self.state == 'PARK_PLAN':
            if now - self.since >= SETTLE_TIME:
                self._plan(now, (x, y, th))
            return 0.0, 0.0
        if self.state == 'PARK_SETTLE':
            if now - self.since >= SETTLE_TIME:
                self._settled(now, (x, y, th))
            return 0.0, self.current[0]
        if self.state == 'PARK_STEER':
            if now - self.since >= self.wait:
                self.state = 'PARK_MOVE'
            return self.current[1] * STEER_CREEP, self.current[0]
        if self.state == 'PARK_MOVE':
            return self._drive(now, x, y, th)
        return 0.0, 0.0

    def travelled(self, lidar, turn):
        """Signed distance of this cycle while a move runs or rolls after its stop, or None (the
        mission's own). On an arc it is the gyro's heading change over the curvature: deep in the
        lot the LiDAR sees little more than lines along x (the near limit is inside its minimum
        range), so its along-track distance drifts; the gyro does not. The gear is the move's,
        also while it rolls on after the stop command."""
        if self.state not in ('PARK_MOVE', 'PARK_SETTLE') or self.current is None:
            return None
        steer, direction = self.current[:2]
        if abs(steer) >= GYRO_ARC * self.steer_limit:
            return direction * abs(turn) * WHEELBASE / abs(math.tan(steer))
        return direction * lidar

    # ------------------------------------------------------------- phases
    def _approach(self, now, x, y, th):
        """Forward along the line until aligned with it and past the planning pose; back
        straight to that pose if the line-up took the car beyond it."""
        line = self.lot.y1 + OUTLINE_HALF_WIDTH + GAP
        target = self.lot.xb + AHEAD
        aligned = abs(y - line) < 0.008 and abs(wrap(th)) < math.radians(2.0)
        if self.state == 'PARK_APPROACH' and x >= target and (aligned or x >= target + RUN_PAST):
            self.state = 'PARK_BACK'
        if self.state == 'PARK_BACK':
            return self._back(now, x, y, th, line, target)
        steer = 2.0 * wrap(math.atan2(line - y, 0.30) - th)
        speed = 2.0 * self.speed if target - x > 0.30 else self.speed
        return speed, max(-self.steer_limit, min(self.steer_limit, steer))

    def _back(self, now, x, y, th, line, target):
        """Reverse along the line: to the planning pose, or to ``back_to`` to approach again."""
        if self.back_to is not None and x <= self.back_to:
            self.back_to, self.state = None, 'PARK_APPROACH'
            return 0.0, 0.0
        if self.back_to is None and x <= target + 0.005:
            self.state, self.since = 'PARK_PLAN', now
            return 0.0, 0.0
        # Reversing: steer against the heading error to keep the line.
        steer = -2.0 * wrap(math.atan2(line - y, -0.30) - th - math.pi)
        return -self.speed, max(-self.steer_limit, min(self.steer_limit, steer))

    def _plan(self, now, pose):
        line = self.lot.y1 + OUTLINE_HALF_WIDTH + GAP
        aligned = abs(pose[1] - line) < ALIGNED[0] and abs(wrap(pose[2])) < ALIGNED[1]
        if self.plans == 0 and not aligned and self.reapproaches < MAX_REAPPROACH:
            self.reapproaches += 1
            self.back_to, self.state = pose[0] - REAPPROACH, 'PARK_BACK'
            self.log.append(dict(event='park_reapproach', pose=list(pose)))
            return
        if self.plans == 0:
            self._load(pose)
            if not self.moves:
                self.state = 'PARK_FAIL'
                return
        self._next(now, pose)

    def _load(self, pose):
        """The nominal moves for this lot, after a straight to the planning pose if needed."""
        self.plans = 1
        shift = self.lot.xb - Lot(1.0).xb      # CW: the same lot shifted along x
        start = self.lot.xb + AHEAD
        if abs(pose[0] - start) > LINE_UP:   # the approach stops a little off: drive to the pose
            self.moves.append((0.0, 1.0 if pose[0] < start else -1.0, abs(pose[0] - start),
                               ('along', (start, pose[1], 0.0))))
        for steer, direction, length, (kind, value) in nominal_moves(self.steer_limit):
            if kind == 'along':
                value = (value[0] + shift, value[1], value[2])
            self.moves.append((steer, direction, length, (kind, value)))
        self.log.append(dict(event='park_plan', pose=list(pose), moves=len(self.moves)))

    def _finishing(self, pose):
        """After the plan: straighten (alternate gears at full lock toward parallel), then centre."""
        x, y, th = pose
        straighten = self._straighten(pose)
        if straighten:
            return straighten
        if not self.centred:
            self.centred = True
            xc = 0.5 * (self.lot.xa + self.lot.xb) - 0.5 * (OUTLINE_FRONT - OUTLINE_REAR)
            return (0.0, 1.0 if x < xc else -1.0, abs(xc - x), ('along', (xc, y, 0.0)))
        return None

    def _straighten(self, pose):
        """Next straightening move (full lock toward parallel in the other gear), or None when
        parallel, out of moves, or two moves in a row barely moved (both guards hold the car)."""
        x, y, th = pose
        if self.straightened and self.current:
            moved = math.hypot(x - self.current[4][0], y - self.current[4][1])
            self.short = self.short + 1 if moved < STUCK else 0
        if abs(wrap(th)) < STRAIGHTEN or self.straightened >= STRAIGHTEN_MOVES or self.short >= 2:
            return None
        self.straightened += 1
        sign = 1.0 if wrap(th) > 0 else -1.0
        direction = -self.current[1] if self.current else 1.0   # the other gear has room
        return (-sign * direction * self.steer_limit, direction, 0.3, ('heading', 0.0))

    def _next(self, now, pose):
        move_ = self.moves.pop(0) if self.moves else self._finishing(pose)
        if move_ is None:
            parked = self.lot.parked(*pose, inside=INSIDE, parallel=PARALLEL)
            self.state = 'PARKED' if parked else 'PARK_FAIL'
            self.log.append(dict(event='park_end', pose=list(pose), parked=parked, moves=self.driven))
            return
        steer, direction, length, target = move_
        previous = self.current[0] if self.current else 0.0
        self.wait = STEER_TIME[0] + STEER_TIME[1] * abs(steer - previous)
        self.current = (steer, direction, length, target, pose)
        self.resumes = 0
        self.driven += 1
        self.state, self.since = 'PARK_STEER', now

    def _reached(self, x, y, th, early):
        """The move's target is reached, or will be after rolling ``early`` metres more."""
        steer, direction, _, (kind, value), _ = self.current
        if kind == 'heading':
            turning = math.copysign(1.0, direction * steer)       # sign of the heading change
            return turning * wrap(th - value) >= -early * abs(math.tan(steer)) / WHEELBASE
        if kind == 'depth':
            down = math.copysign(1.0, direction * math.sin(th))   # sign of the change of y
            return down * (y - value) >= -early * abs(math.sin(th))
        tx, ty, tth = value
        return direction * ((x - tx) * math.cos(tth) + (y - ty) * math.sin(tth)) >= -early

    def _settled(self, now, pose):
        """Stopped after a move: learn how far the car rolled; go on with the move if it stopped
        short of its target (and the guard allows), else the next move."""
        rolled = math.hypot(pose[0] - self.stop_pose[0], pose[1] - self.stop_pose[1])
        self.coast = 0.5 * self.coast + 0.5 * min(rolled, 2.0 * COAST)
        steer, direction = self.current[:2]
        ahead = move(*pose, steer, direction * GUARD_AHEAD)
        if (not self._reached(*pose, REACH_SLACK) and self.resumes < MAX_RESUME
                and self.lot.free(*ahead, steer, GUARD_MARGIN)):
            self.resumes += 1
            self.state = 'PARK_MOVE'
            return
        self._next(now, pose)

    def _drive(self, now, x, y, th):
        steer, direction, length, _, (x0, y0, _) = self.current
        overrun = math.hypot(x - x0, y - y0) >= length + OVERRUN
        ahead = move(x, y, th, steer, direction * GUARD_AHEAD)
        if overrun or not self.lot.free(*ahead, steer, GUARD_MARGIN):
            self.resumes = MAX_RESUME          # stopped by the guard or too long: do not go on
        elif not self._reached(x, y, th, self.coast):
            return direction * self.speed, steer
        self.stop_pose, self.state, self.since = (x, y, th), 'PARK_SETTLE', now
        return 0.0, steer
        return direction * self.speed, steer
