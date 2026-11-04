"""Parallel parking into the lot (rules: start section, 20 cm deep, 1.5 x robot length).

Track frame (obstacle_field): start straight at the bottom, outer wall at y = -OUTER,
travel +x. The lot is at the CCW-downstream end of the start straight, one limit next
to the section line: x in [0.48 - L, 0.48] in CCW, mirrored to [-0.48, -0.48 + L] in CW.
The limits are 2 cm magenta plates sticking 20 cm out of the outer wall; the lidar sees
them and refines the lot position. In both directions the car arrives heading +x and
reverses into the lot behind it.

States:
  PARK_LOCATE    stopped: average the limit positions over a few scans
  PARK_APPROACH  forward, parallel to the wall, to a pose ahead of the lot
  PARK_BACK      reverse straight to the entry point if the lot was right at the start
  PARK_ENTRY     reverse at full lock toward the wall until ~50 deg (fixed first move)
  PARK_DECIDE    stopped: keep the planned arcs if they still work from the estimate,
                 else plan again (A* over 3 cm arcs, outline + steered wheels)
  PARK_STEER     creep a few mm so the servo reaches the next arc's angle
  PARK_MOVE      drive the arcs; stop only to change gear or steering, or if the plan breaks
  PARKED         inside the lot and parallel (own estimate)
  PARK_FAIL      no plan
"""
import heapq
import math
import time

from .car import (FRONT_TRACK, LENGTH, OUTLINE_FRONT, OUTLINE_HALF_WIDTH as HW, OUTLINE_REAR,
                                    REAR_AXLE_X, WHEEL_RADIUS, WHEEL_WIDTH)

from .field import OUTER, WHEELBASE, scan_points, wrap

LOT_DEPTH = .20
LIMIT_THICKNESS = .02
LOT_LENGTH = 1.5*LENGTH
LOT_END = .48                 # m, inner face of the limit next to the section line
FRONT_AXLE_X, HALF_TRACK, WHEEL_HALF_WIDTH = WHEELBASE/2, FRONT_TRACK/2, WHEEL_WIDTH/2
APPROACH_SPEED = .15          # m/s
APPROACH_RUN = .8             # m driven at least while approaching (room to line up)
APPROACH_EXTRA = .3           # m allowed past the target to finish lining up
PRE_GAP = .05                 # m between the car side and the lot line while approaching
PRE_AHEAD = .41               # m from the lot's lower x end to the car centre before reversing
ENTRY_ANGLE = math.radians(50.)  # heading at the end of the fixed entry arc
ENTRY_MAX = .35               # m of entry arc at most
CREEP_SPEED = .01             # m/s while the servo turns (a few mm)
CREEP_TIME = .3               # s
SETTLE_TIME = .3              # s stopped before deciding
LOCATE_SCANS = 5
MAX_PLANS = 30
STEP = .03                    # m per arc
SUBSTEPS = 3
GEAR_CHANGE = .05             # m-equivalent cost of a direction change
SEARCH_WEIGHTS = (3., 4., 2.)  # weighted A* portfolio, tried in order within the budget
SEARCH_LIMIT = 200000         # expansions per attempt
SEARCH_BUDGET = 10.           # s of computation per plan (all attempts)
NUDGE = .02                   # m moved straight when no plan is found, before retrying
FALLBACK_MARGINS = (.01, .005)  # m, retried when no plan keeps park_margin
GOAL_INSIDE = .015            # m the search keeps inside every side of the lot
PARKED_INSIDE = .005          # m required to declare PARKED from the own estimate
GUARD_MARGIN = .005           # m; closer than this to wall/limits while moving -> stop
VALID_SLACK = .005            # m of the planning margin the executed path may use up
STEERS = (-1., -.5, 0., .5, 1.)  # fraction of the steering limit
REACH = math.hypot(max(OUTLINE_FRONT, OUTLINE_REAR), HW+.02)   # m, outline radius incl. steered wheels


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


class ObstacleParking:
    """Drives the parking phases on top of ObstacleDriver's localization."""

    def __init__(self, cfg, driver):
        self.cfg = cfg
        self.driver = driver
        self.lot = Lot(driver.sign)
        self.state = 'PARK_LOCATE'
        self.limit_seen = {0: [], 1: []}
        self.scans = 0
        self.timer = 0.
        self.last_now = None
        self.pre = None
        self.entry = 0.
        self.entry_x = None
        self.current = None   # (steer, direction) of the arc being driven
        self.gear = None      # 0 forward, 1 reverse: last arc's direction
        self.left = 0.
        self.path = []        # planned arcs
        self.plan_margin = cfg.park_margin
        self.plans = 0
        self.log = []         # events for the recording

    def snapshot(self):
        return dict(state=self.state, lot=[self.lot.xa, self.lot.xb], plans=self.plans,
                    arc=None if self.current is None else list(self.current), arcs_left=len(self.path))

    # ------------------------------------------------------------- control
    def step(self, now, yaw_abs, distance, scan):
        """-> (speed, steer, state); steer in the real frame. scan: new scan or None."""
        dt = 0. if self.last_now is None else max(0., min(.1, now-self.last_now))
        self.last_now = now
        if not self.driver.track(yaw_abs, distance):
            return self._out(0., 0.)
        pose = self.driver.pose()
        x, y, th = pose
        if self.state == 'PARK_LOCATE':
            if scan is not None:
                self._locate(scan, pose)
            if self.scans >= LOCATE_SCANS:
                self._fix_lot()
                # Entry point ahead of the lot; line up over at least APPROACH_RUN first,
                # then reverse straight back to it if it is behind (lot right at the start).
                self.entry_x = self.lot.xa+PRE_AHEAD
                self.pre = (max(self.entry_x, x+APPROACH_RUN), self.lot.y1+HW+PRE_GAP)
                self.state = 'PARK_APPROACH'
            return self._out(0., 0.)
        if self.state == 'PARK_APPROACH':
            return self._approach(x, y, th)
        if self.state == 'PARK_BACK':
            # Reverse straight (hold the line y = ty, heading 0) to the entry point.
            if x <= self.entry_x:
                self.state, self.timer, self.entry = 'PARK_ENTRY', 0., 0.
                return self._out(0., 0.)
            ty = self.pre[1]
            steer = max(-self.cfg.steer_limit, min(self.cfg.steer_limit, -2.*wrap(math.atan2(ty-y, -.25)-th-math.pi)))
            speed = APPROACH_SPEED if x-self.entry_x > .2 else .05
            return self._out(-speed, steer)
        if self.state == 'PARK_ENTRY':
            steer = -self.cfg.steer_limit   # reversing: the rear swings toward the wall
            self.entry += distance
            ahead = move(x, y, th, steer, -.015)
            if (th >= ENTRY_ANGLE or self.entry >= ENTRY_MAX
                    or not self.lot.free(*ahead, steer, GUARD_MARGIN)):
                self.log.append(dict(event='entered', pose=list(pose), length=self.entry))
                self.gear = 1
                self.state, self.timer = 'PARK_DECIDE', 0.
                return self._out(0., 0.)
            if self.timer < CREEP_TIME:
                self.timer += dt
                return self._out(-CREEP_SPEED, steer)
            return self._out(-self.cfg.park_speed, steer)
        if self.state == 'PARK_DECIDE':
            self.timer += dt
            if self.timer < SETTLE_TIME:
                return self._out(0., 0.)
            if self.lot.parked(x, y, th, inside=PARKED_INSIDE):
                self.state = 'PARKED'
                self.log.append(dict(event='parked', pose=list(pose), plans=self.plans))
                return self._out(0., 0.)
            arc = self._decide(pose)
            if arc is None:
                return self._out(0., 0.)
            self.current = arc
            self.left, self.state, self.timer = STEP, 'PARK_STEER', 0.
        if self.state == 'PARK_STEER':
            steer, direction = self.current
            self.timer += dt
            if self.timer >= CREEP_TIME:
                self.state = 'PARK_MOVE'
            return self._out(direction*CREEP_SPEED, steer)
        if self.state == 'PARK_MOVE':
            steer, direction = self.current
            self.left -= distance
            ahead = move(x, y, th, steer, direction*.015)
            if not self.lot.free(*ahead, steer, GUARD_MARGIN):
                self.log.append(dict(event='park_guard', pose=list(pose)))
                self.path = []
                self.state, self.timer = 'PARK_DECIDE', 0.
                return self._out(0., 0.)
            if self.left <= .002:
                self.gear = 0 if direction > 0 else 1
                if self.path:
                    self.path.pop(0)
                if (not self.lot.parked(x, y, th, inside=PARKED_INSIDE) and self.path
                        and tuple(self.path[0]) == tuple(self.current) and self._still_valid(pose)):
                    self.left += STEP          # same arc and the plan still works: keep rolling
                else:
                    self.state, self.timer = 'PARK_DECIDE', 0.
                    return self._out(0., 0.)
            return self._out(direction*self.cfg.park_speed, steer)
        return self._out(0., 0.)

    def _out(self, speed, steer):
        self.driver.speed = speed
        return speed, self.driver.sign*steer, self.state

    # ------------------------------------------------------------ planning
    def _decide(self, pose):
        """Next arc: keep the plan if it still works from the estimate, else replan."""
        if self.path and self._still_valid(pose):
            return tuple(self.path[0])
        self.plans += 1
        path = None
        expanded = 0
        if self.plans <= MAX_PLANS:
            deadline = time.monotonic()+SEARCH_BUDGET
            margins = (self.cfg.park_margin,)+tuple(m for m in FALLBACK_MARGINS if m < self.cfg.park_margin)
            attempts = [(m, w) for m in margins for w in SEARCH_WEIGHTS]
            for k, (margin, weight) in enumerate(attempts):
                # Start already closer than the margin (overshoot): use what is left.
                while margin > .001 and not self.lot.free(*pose, 0., margin):
                    margin = max(.001, margin-.002)
                # Share what is left of the budget among the remaining attempts.
                share = (deadline-time.monotonic())/(len(attempts)-k)
                path, used = search(self.lot, pose, self.gear, self.cfg.steer_limit, margin,
                                    time.monotonic()+max(share, .2), weight)
                expanded += used
                if path:
                    self.plan_margin = margin
                    break
        self.log.append(dict(event='park_plan', pose=list(pose), expanded=expanded,
                             arcs=None if path is None else len(path), margin=self.plan_margin))
        self.path = path or []
        if not self.path and self.plans < MAX_PLANS:
            # No plan in time: move a little where it is free and try again from there.
            for direction in (1., -1.):
                if self.lot.arc(pose, 0., direction, .005, NUDGE) is not None:
                    self.path = []
                    return (0., direction)
        if not self.path:
            self.state = 'PARK_FAIL'
            self.log.append(dict(event='park_fail', pose=list(pose)))
            return None
        return tuple(self.path[0])

    def _still_valid(self, pose):
        margin = max(.001, self.plan_margin-VALID_SLACK)
        for steer, direction in self.path:
            pose = self.lot.arc(pose, steer, direction, margin)
            if pose is None:
                return False
        return self.lot.parked(*pose, inside=PARKED_INSIDE)

    # ------------------------------------------------------------- phases
    def _approach(self, x, y, th):
        tx, ty = self.pre
        aligned = abs(y-ty) < .02 and abs(wrap(th)) < math.radians(2.)
        if x >= tx and (aligned or x >= tx+APPROACH_EXTRA):
            self.log.append(dict(event='approached', pose=[x, y, th]))
            self.state, self.timer, self.entry = ('PARK_BACK' if x > self.entry_x+.02 else 'PARK_ENTRY'), 0., 0.
            return self._out(0., 0.)
        # Pursue the line y = ty so the car arrives parallel to the wall.
        desired = math.atan2(ty-y, .25)
        steer = max(-self.cfg.steer_limit, min(self.cfg.steer_limit, 2.*wrap(desired-th)))
        speed = APPROACH_SPEED if tx-x > .25 else (.5*APPROACH_SPEED if tx-x > .08 else .05)
        return self._out(speed, steer)

    # ------------------------------------------------------- lot location
    def _locate(self, scan, pose):
        """Limit plates: returns sticking out of the outer wall near the expected x."""
        bx, by = scan_points(scan, self.driver.sign)
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        px, py = x+c*bx-s*by, y+s*bx+c*by
        expected = (self.lot.xa-LIMIT_THICKNESS/2, self.lot.xb+LIMIT_THICKNESS/2)
        for k, ex in enumerate(expected):
            near = [qx for qx, qy in zip(px, py)
                    if abs(qx-ex) < .10 and -OUTER+.03 < qy < -OUTER+LOT_DEPTH+.01]
            if len(near) >= 3:
                mx = sum(near)/len(near)
                # The lidar sees the face toward the car: shift to the plate centre.
                mx += LIMIT_THICKNESS/2 if mx > x else -LIMIT_THICKNESS/2
                self.limit_seen[k].append(mx)
        self.scans += 1

    def _fix_lot(self):
        seen = {k: sorted(v)[len(v)//2] for k, v in self.limit_seen.items() if v}
        if 0 in seen:
            self.lot.xa = seen[0]+LIMIT_THICKNESS/2
        if 1 in seen:
            self.lot.xb = seen[1]-LIMIT_THICKNESS/2
        self.log.append(dict(event='lot', xa=self.lot.xa, xb=self.lot.xb,
                             seen={k: len(v) for k, v in self.limit_seen.items()}))
