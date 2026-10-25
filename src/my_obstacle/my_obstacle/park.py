"""Parallel parking after the three laps: stop, plan, then drive the plan slowly, move by move.

The lot is 1.5 car lengths (0.35 m) by 0.20 m against the outer wall of the start straight; with
the measured car (0.233 m) a single reverse manoeuvre would need ~0.42 m, so parking takes
14-18 short moves. The car may stop as long as it needs (only the 3 minutes count), so:

1. APPROACH: forward along a line 5 cm off the lot, to a pose 10 cm past its far limit.
2. PLAN: stopped, a weighted A* over 3 cm full/half-lock arcs (``search`` below, the
   measured outline and the steered wheels, 2 cm from the limits and the wall) from the pose.
3. STEER / MOVE: each run of equal arcs is one move: stop, turn the wheels, drive slowly until
   the planned heading change (gyro) or distance is done, or the guard sees the outline 2 cm
   ahead closer than 8 mm to a limit or the wall.
4. After the last move: parked (inside, parallel) or plan again from where the car is; after
   ``MAX_PLANS`` it stops where it is (partly parked still scores 7).

Frame: the v2 track frame (start straight at the bottom, travel +x, outer wall at y = -1.5; CW
rounds mirrored). The lot is at x in [0.48 - L, 0.48] in CCW and [-0.48, -0.48 + L] in CW; the
geometry seen from the approach line is the same in both, only shifted along x.
"""
import heapq
import math
import time
from functools import lru_cache

from robotek_common.vehicle import (FRONT_TRACK, LENGTH, OUTLINE_FRONT, OUTLINE_HALF_WIDTH, OUTLINE_REAR,
                                    REAR_AXLE_X, WHEEL_RADIUS, WHEEL_WIDTH, WHEELBASE)

from .lane import OUTER, wrap

HW = OUTLINE_HALF_WIDTH
LOT_DEPTH = .20
LIMIT_THICKNESS = .02
LOT_LENGTH = 1.5*LENGTH
LOT_END = .48                 # m, inner face of the limit next to the section line
FRONT_AXLE_X, HALF_TRACK, WHEEL_HALF_WIDTH = WHEELBASE/2, FRONT_TRACK/2, WHEEL_WIDTH/2
STEP = .03                    # m per arc of the search
SUBSTEPS = 3
GEAR_CHANGE = .05             # m-equivalent cost of a direction change
SEARCH_LIMIT = 200000         # expansions per search
GOAL_INSIDE = .015            # m the search keeps inside every side of the lot
STEERS = (-1., -.5, 0., .5, 1.)  # fraction of the steering limit
REACH = math.hypot(max(OUTLINE_FRONT, OUTLINE_REAR), HW+.02)   # m, outline radius incl. steered wheels


# ---------------------------------------------------------------- geometry and search
def footprint(x, y, th, front=OUTLINE_FRONT, half_width=HW, rear=OUTLINE_REAR):
    """Rectangle of the car: `front` ahead of the pose and `rear` behind it (default: the overall outline)."""
    c, s = math.cos(th), math.sin(th)
    return [(x+c*u-s*v, y+s*u+c*v) for u, v in ((front, half_width), (front, -half_width),
                                                  (-rear, -half_width), (-rear, half_width))]


def front_wheels(x, y, th, steer):
    """Steered front wheels (they stick out of the outline by up to ~1.2 cm at 25 deg)."""
    wheels = []
    for side in (1., -1.):
        u, v = FRONT_AXLE_X, side*HALF_TRACK
        cx, cy = x+math.cos(th)*u-math.sin(th)*v, y+math.sin(th)*u+math.cos(th)*v
        wheels.append(footprint(cx, cy, th+steer, WHEEL_RADIUS, WHEEL_HALF_WIDTH, WHEEL_RADIUS))
    return wheels


def _overlaps(poly, x0, x1, y0, y1):
    """Convex polygon versus axis-aligned box (separating axes)."""
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    if max(xs) < x0 or min(xs) > x1 or max(ys) < y0 or min(ys) > y1:
        return False
    box = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    for i in range(4):
        (ax, ay), (bx, by) = poly[i], poly[(i+1) % 4]
        nx, ny = ay-by, bx-ax
        a = [nx*px+ny*py for px, py in poly]
        b = [nx*px+ny*py for px, py in box]
        if max(a) < min(b) or max(b) < min(a):
            return False
    return True


def move(x, y, th, steer, ds):
    """Bicycle model about the rear axle; (x, y) is base_link."""
    rx, ry = x+REAR_AXLE_X*math.cos(th), y+REAR_AXLE_X*math.sin(th)
    dth = ds*math.tan(steer)/WHEELBASE
    mid = th+dth/2
    rx, ry, th = rx+ds*math.cos(mid), ry+ds*math.sin(mid), th+dth
    return rx-REAR_AXLE_X*math.cos(th), ry-REAR_AXLE_X*math.sin(th), th


class Lot:
    """Lot interior [xa, xb] x [-OUTER, -OUTER + LOT_DEPTH] in the track frame."""

    def __init__(self, sign):
        if sign > 0:
            self.xa, self.xb = LOT_END-LOT_LENGTH, LOT_END
        else:
            self.xa, self.xb = -LOT_END, -LOT_END+LOT_LENGTH
        self.y0, self.y1 = -OUTER, -OUTER+LOT_DEPTH

    def limits(self, margin=0.):
        """Boxes of the two limits, grown by margin."""
        return [(self.xa-LIMIT_THICKNESS-margin, self.xa+margin, self.y0, self.y1+margin),
                (self.xb-margin, self.xb+LIMIT_THICKNESS+margin, self.y0, self.y1+margin)]

    def parked(self, x, y, th, inside=0., parallel=math.radians(3.)):
        return abs(wrap(th)) <= parallel and all(
            self.xa+inside <= px <= self.xb-inside and self.y0+inside <= py <= self.y1-inside
            for px, py in footprint(x, y, th))

    def free(self, x, y, th, steer, margin):
        # Quick accept: the whole car (radius REACH) clear of wall and limits.
        if y-REACH > self.y1+margin or (y-REACH > self.y0+margin and (
                x+REACH < self.xa-LIMIT_THICKNESS-margin or x-REACH > self.xb+LIMIT_THICKNESS+margin)):
            return True
        shapes = [footprint(x, y, th)]+(front_wheels(x, y, th, steer) if steer else [])
        boxes = self.limits(margin)
        for poly in shapes:
            if min(p[1] for p in poly) < self.y0+margin or any(_overlaps(poly, *box) for box in boxes):
                return False
        return True

    def arc(self, pose, steer, direction, margin, length=STEP):
        """End pose of an arc, or None if it collides."""
        x, y, th = pose
        steps = max(1, round(SUBSTEPS*length/STEP))
        for _ in range(steps):
            x, y, th = move(x, y, th, steer, direction*length/steps)
            if not self.free(x, y, th, steer, margin):
                return None
        return x, y, th


def search(lot, pose, gear, steer_limit, margin, deadline, weight=3.):
    """Weighted A* over STEP arcs to a pose GOAL_INSIDE the lot -> ([(steer, dir)], expanded)."""
    cx, cy = (lot.xa+lot.xb)/2, (lot.y0+lot.y1)/2
    queue = [(0., 0., pose, gear, ())]
    seen = set()
    while queue and len(seen) < SEARCH_LIMIT and time.monotonic() < deadline:
        _, cost, (x, y, th), last, path = heapq.heappop(queue)
        key = (round(x/.01), round(y/.01), round(th/math.radians(3.)), last)
        if key in seen:
            continue
        seen.add(key)
        if lot.parked(x, y, th, inside=GOAL_INSIDE):
            return list(path), len(seen)
        for fraction in STEERS:
            steer = fraction*steer_limit
            for direction in (1., -1.):
                end = lot.arc((x, y, th), steer, direction, margin)
                if end is None:
                    continue
                g = 0 if direction > 0 else 1
                step_cost = STEP+(GEAR_CHANGE if last is not None and last != g else 0.)
                h = math.hypot(end[0]-cx, end[1]-cy)+.1*abs(wrap(end[2]))
                heapq.heappush(queue, (cost+step_cost+weight*h, cost+step_cost, end, g,
                                       path+((steer, direction),)))
    return None, len(seen)


# ---------------------------------------------------------------- the manoeuvre
GAP = 0.04                  # m between the car side and the lot line while approaching
RUN_PAST = 0.40             # m past the planning pose the car may drive to line up
AHEAD = 0.05                # m from the lot's far limit to base_link where the car stops to plan
PLAN_MARGINS = (0.015, 0.01)  # m kept from the limits and the wall by the plan (first that works)
GUARD_MARGIN = 0.008        # m; the outline 2 cm ahead closer than this ends a move
GUARD_AHEAD = 0.02
PLAN_BUDGET = 60.0          # s per margin at most; the search stops at its expansion limit first
MAX_PLANS = 10
STEER_TIME = (0.2, 1.0 / 1.2)  # s stopped for the servo: delay + its 1.2 rad/s sweep to the next angle
COAST = 0.012              # m the car rolls after a stop command (motor lag at park speed)
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


class Parking:
    """PARK_APPROACH -> PARK_PLAN -> (PARK_STEER -> PARK_MOVE)* -> PARKED or PARK_FAIL."""

    def __init__(self, mission, speed, margin):
        self.speed = speed
        self.steer_limit = mission.p.steer_limit
        self.lot = Lot(1.0 if mission.selected_direction == 'ccw' else -1.0)
        self.state = 'PARK_APPROACH'
        self.moves = []             # [(steer, direction, length)]
        self.current = None         # (steer, direction, length, start pose)
        self.since = None
        self.plans = 0
        self.gear = None
        self.log = []

    def step(self, now, x, y, th):
        """-> (speed, steer) in the track frame."""
        if self.state in ('PARK_APPROACH', 'PARK_BACK'):
            return self._approach(now, x, y, th)
        if self.state == 'PARK_PLAN':
            if now - self.since >= SETTLE_TIME:
                self._plan(now, (x, y, th))
            return 0.0, 0.0
        if self.state == 'PARK_STEER':
            if now - self.since >= self.wait:
                self.state = 'PARK_MOVE'
            return 0.0, self.current[0]
        if self.state == 'PARK_MOVE':
            return self._drive(now, x, y, th)
        return 0.0, 0.0

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
            if x <= target + 0.005:
                self.state, self.since = 'PARK_PLAN', now
                return 0.0, 0.0
            # Reversing: steer against the heading error to keep the line.
            steer = -2.0 * wrap(math.atan2(line - y, -0.30) - th - math.pi)
            return -self.speed, max(-self.steer_limit, min(self.steer_limit, steer))
        steer = 2.0 * wrap(math.atan2(line - y, 0.30) - th)
        speed = 2.0 * self.speed if target - x > 0.30 else self.speed
        return speed, max(-self.steer_limit, min(self.steer_limit, steer))

    def _plan(self, now, pose):
        if self.lot.parked(*pose, inside=INSIDE, parallel=PARALLEL):
            self.state = 'PARKED'
            return
        if self.plans >= MAX_PLANS:
            self.state = 'PARK_FAIL'
            return
        self.plans += 1
        path, expanded = None, 0
        if self.plans == 1:
            path = list(nominal_plan(self.steer_limit)) or None
        for margin in () if path else PLAN_MARGINS:
            while margin > 0.004 and not self.lot.free(*pose, 0.0, margin):
                margin -= 0.002      # overshoot: plan with what is left
            path, used = search(self.lot, pose, self.gear, self.steer_limit, margin,
                                time.monotonic() + PLAN_BUDGET, 2.0)
            expanded += used
            if path:
                break
        self.log.append(dict(event='park_plan', pose=list(pose), arcs=len(path or []), expanded=expanded))
        if not path:
            self.state = 'PARK_FAIL'
            return
        self.moves = []
        for steer, direction in path:
            if self.moves and self.moves[-1][:2] == (steer, direction):
                self.moves[-1] = (steer, direction, self.moves[-1][2] + STEP)
            else:
                self.moves.append((steer, direction, STEP))
        self._next(now, pose)

    def _next(self, now, pose):
        if not self.moves:
            self.state, self.since = 'PARK_PLAN', now
            return
        steer, direction, length = self.moves.pop(0)
        previous = self.current[0] if self.current else 0.0
        self.wait = STEER_TIME[0] + STEER_TIME[1] * abs(steer - previous)
        self.current = (steer, direction, length, pose)
        self.gear = 0 if direction > 0 else 1
        self.state, self.since = 'PARK_STEER', now

    def _drive(self, now, x, y, th):
        steer, direction, length, (x0, y0, th0) = self.current
        if abs(steer) > 1e-6:
            # Heading change of the arc: the gyro measures it better than the distance.
            done = abs(wrap(th - th0)) >= (length - COAST) * abs(math.tan(steer)) / WHEELBASE
        else:
            done = math.hypot(x - x0, y - y0) >= length - COAST
        ahead = move(x, y, th, steer, direction * GUARD_AHEAD)
        if done or not self.lot.free(*ahead, steer, GUARD_MARGIN):
            if not done:
                self.moves = []      # the plan no longer fits: plan again from here
            self._next(now, (x, y, th))
            return 0.0, steer
        return direction * self.speed, steer
