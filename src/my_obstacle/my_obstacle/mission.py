"""The mission (step 7): SETUP -> RUN -> (PARK) -> STOP.

SETUP   stopped: calibrate the gyroscope (0.3 s), then read the direction of the round from the
        walls of the first scans and place the car on the start straight (``localizer``).
RUN     every tick: update the pose, file the signs of a new scan by seat (``signs``), plan the
        line (``plan`` + ``corner``) and follow it (``follow``). Three extra behaviours:
        * Start: stop 0.25 s to file the first signs, and back up straight when the first sign is
          too close to reach its side going forward (``_start``);
        * a sign whose colour is still unknown: stop short of it, back up at full lock so the nose
          (and the camera) turns 15 degrees toward it, and look (``_look``);
        * a front closer than 0.16 m: back up 0.8 s (``_recover``).
PARK    with ``park`` on, after three laps: the parking manoeuvres (``park``, step 8).
STOP    after three laps and 0.7 units into the start straight (or when parked).

Everything is in the track frame (``lane``): a CW round is mirrored, so the steering this class
returns is multiplied by the direction's sign before it reaches the wheels.
"""
from dataclasses import dataclass, fields
import math

import numpy as np

from robotek_common.vehicle import OUTLINE_HALF_WIDTH

from . import corner, follow, lane, plan
from .localizer import Localizer, direction_from_scan, scan_points
from .signs import SeatMemory

CALIBRATE_SECONDS = 0.3
DIRECTION_MARGIN = (0.10, 0.03)  # wall-fit margin to pick the direction: at once, and after the wait
DIRECTION_WAIT = 1.0        # s of scans (the parking lot limits clutter the outer wall in CCW)
SIDE_CENTER = math.pi / 2
SIDE_HALF_WIDTH = 0.44
FRONT_HALF_WIDTH = 0.35
RECOVER_SPEED = 0.15
RECOVER_SECONDS = 0.8
START_LOOK = (0.25, 1.0)    # s stopped at Start to file the first signs; at most, for a near colour
BACKUP_LIMIT = 0.5          # m the car may back up at Start
PEEK_AT = (0.6, 1.4)        # track units to an unknown sign where the car stops to look at it
PEEK_TURN = math.radians(15.0)  # rad the nose turns toward the sign, backing up at full lock
PEEK_LOOK = 0.5             # s stopped looking


@dataclass
class Params:
    cruise_speed: float = 0.35      # m/s on a clear lane
    pillar_speed: float = 0.30      # m/s while passing a sign or changing side
    finish_speed: float = 0.15      # m/s on the last metres and the stop
    pillar_clearance: float = 0.18  # m from a sign to the car's side on a straight
    pillar_lookahead: float = 1.80  # m ahead a sign is looked for
    steer_kp: float = 1.5           # rad of steer per rad of heading error to the planned line
    steer_kd: float = 0.2           # gyro damping
    steer_limit: float = 0.307      # rad, the real car after per-side compensation (17.6 deg)
    stop_progress: float = 24.7     # track units: 3 laps + 0.7 into the start straight
    park: float = 0.0               # 1: park in the lot after the three laps
    park_after: float = 24.0        # track units driven before parking (3 laps; 0 in parking-only tests)
    park_speed: float = 0.06        # m/s while manoeuvring in the lot
    park_margin: float = 0.015      # m kept from the limits and the wall by the parking search
    scan_max_age: float = 0.35      # s; an older scan brakes


def make_params(values=None):
    known = {f.name for f in fields(Params)}
    unknown = set(values or {}) - known
    if unknown:
        raise ValueError('Unsupported parameters: ' + ', '.join(sorted(unknown)))
    return Params(**{k: float(v) for k, v in (values or {}).items()})


def _window(scan, center, half):
    """Valid ranges within +-half of a bearing, in the car frame."""
    ranges = np.asarray(scan.ranges, dtype=float)
    if ranges.size == 0:
        return np.empty(0)
    angle = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
    relative = np.arctan2(np.sin(angle - center), np.cos(angle - center))
    valid = np.isfinite(ranges) & (ranges >= max(0.16, scan.range_min)) & (ranges <= scan.range_max)
    return ranges[(np.abs(relative) <= half) & valid]


class Mission:
    """The decision chain; ``strategy.Strategy`` (and through it the node and the lab) feeds it."""

    def __init__(self, params=None):
        self.p = make_params(params)
        self.loc = Localizer()
        self.memory = SeatMemory()
        self.state = 'SETUP'
        self.selected_direction = ''
        self.gyro_bias = 0.0
        self.yaw_zero = 0.0
        self._calib = [0.0, 0.0, 0]
        self._last_now = None
        self._scan_stamp = -math.inf
        self._observations = []
        self._obs_stamp = -math.inf
        self._clusters = []
        self._scan = None
        self.loc_th0 = 0.0
        self.pose = (math.nan, math.nan, math.nan)
        self.phi = 0.0
        self.speed = 0.0
        self.steer = 0.0
        self.pillar = None
        self.profile = []           # (progress, lateral offset) knots of the planned line
        self._last_correct = -math.inf
        self._remembered_scan = -math.inf
        self._recover_until = None
        self._direction_margin = 0.0
        self._direction_wait = 0.0
        self._run_since = None
        self._backup_to = None      # progress to back up to at Start; -inf once under way
        self._peeked = set()        # seats already looked at
        self._peek = None           # [seat, phase, yaw at start, time, side]
        self.parking = None
        if self.p.park:
            from . import park
            park.nominal_plan(self.p.steer_limit)   # before the round: a few seconds

    # ------------------------------------------------------------- inputs
    def observe(self, observations, stamp):
        """Camera observations paired with LiDAR clusters: dict(x, y, color) in the car frame."""
        self._observations = observations or []
        self._obs_stamp = stamp

    def step(self, now, scan, scan_stamp, clusters, yaw, gyro, distance):
        """One tick -> (speed, steer, state). ``clusters`` are those of ``scan``; ``yaw`` is the
        integrated gyro, ``gyro`` the raw rate, ``distance`` the LiDAR odometry since last tick."""
        dt = 0.0 if self._last_now is None else max(0.0, min(0.1, now - self._last_now))
        self._last_now = now
        gyro -= self.gyro_bias
        new_scan = scan is not None and scan_stamp > self._scan_stamp
        if new_scan:
            self._scan = scan
            self._scan_stamp = scan_stamp
            self._clusters = clusters
        if self.state == 'SETUP':
            return self._setup(dt, scan, new_scan, yaw, gyro)
        if self.state == 'STOP':
            return 0.0, 0.0, 'STOP'
        if self.state == 'PARK':
            return self._park(now, yaw, gyro, distance)
        return self._run(now, yaw, gyro, distance)

    def sign(self):
        return 1.0 if self.selected_direction == 'ccw' else -1.0

    # -------------------------------------------------------------- setup
    def _setup(self, dt, scan, new_scan, yaw, gyro):
        if self._calib[0] < CALIBRATE_SECONDS:
            self._calib[0] += dt
            self._calib[1] += gyro
            self._calib[2] += 1
            if self._calib[0] >= CALIBRATE_SECONDS:
                self.gyro_bias = self._calib[1] / max(self._calib[2], 1)
                self.yaw_zero = yaw
            return 0.0, 0.0, 'SETUP'
        if new_scan and scan is not None:
            sign, margin = direction_from_scan(scan)
            self._direction_margin = margin
            self._direction_wait += dt
            needed = DIRECTION_MARGIN[0] if self._direction_wait < DIRECTION_WAIT else DIRECTION_MARGIN[1]
            if margin >= needed and sign:
                self.selected_direction = 'ccw' if sign > 0 else 'cw'
                self.loc.initialize(*scan_points(scan, sign), 0.0)
                self.loc_th0 = self.loc.th
                x, y = self.loc.x, self.loc.y
                self.pose = (x, y, self.loc.th)
                self.phi = float(lane.track_s(x, y))
                self.state = 'RUN'
        return 0.0, 0.0, 'SETUP'

    # ---------------------------------------------------------------- run
    def _run(self, now, yaw, gyro, distance):
        sign = self.sign()
        x, y, th = self._update_pose(distance, yaw, gyro, sign)
        _, _, front = self._walls(sign)
        if self._scan_stamp > self._remembered_scan:
            self._remembered_scan = self._scan_stamp
            self.memory.remember(self._clusters, self._observations, self._obs_stamp, self._scan_stamp,
                                 (x, y, th), self.phi, sign, self.p.pillar_lookahead)
        self.pillar = self.memory.next_sign(self.phi, self.p.pillar_lookahead)
        self.profile = self._plan()
        if self.p.park and self.phi >= self.p.park_after:
            from . import park
            self.state = 'PARK'
            self.parking = park.Parking(self, self.p.park_speed, self.p.park_margin)
            return 0.0, 0.0, 'PARK'
        remaining = self.p.stop_progress - self.phi
        if remaining <= 0.0 and not self.p.park:
            self.state = 'STOP'
            return 0.0, 0.0, 'STOP'
        if now - self._scan_stamp > self.p.scan_max_age:
            self.speed = 0.0
            return 0.0, 0.0, 'RUN'
        start = self._start(now) or self._look(now, th)
        if start is not None:
            return start
        steer = sign * follow.steer((x, y, th), self.phi, gyro, self.offset_at,
                                    self.p.steer_kp, self.p.steer_kd, self.p.steer_limit)
        speed = self._speed(remaining, self.offset_at(self.phi), front)
        return self._recover(now, front, speed, steer)

    def _plan(self):
        signs = plan.with_offsets(self.memory.ahead(self.phi, 1.5, plan.HORIZON + 1.0),
                                  self.p.pillar_clearance + OUTLINE_HALF_WIDTH)
        lot = self.selected_direction if self.p.park else None
        return corner.merge(plan.sign_knots(signs, self.phi), signs, self.phi, plan.HORIZON, lot)

    def offset_at(self, progress):
        """Planned offset at a progress value; beside the parking lot never below ``LOT_OFFSET``."""
        offset = plan.offset_at(self.profile, progress)
        if self.p.park and plan.lot_zone(progress, self.selected_direction):
            return max(offset, plan.LOT_OFFSET)
        return offset

    def _start(self, now):
        """At Start: stop to read the first signs, then back up straight when the first sign is
        too close to reach its side going forward (a sign 0.25 m ahead, say)."""
        if self._backup_to == -math.inf:
            return None
        if self._run_since is None:
            self._run_since = now
        if self._backup_to is None:
            waited = now - self._run_since
            near = self.pillar is not None and self.pillar[1] - self.phi < 0.5
            if waited < START_LOOK[0] or (near and self.pillar[2] is None and waited < START_LOOK[1]):
                return 0.0, 0.0, 'RUN'
            self._backup_to = self.phi - self._backup()
        if self.phi > self._backup_to:
            self.speed = -RECOVER_SPEED
            return -RECOVER_SPEED, 0.0, 'RUN'
        self._backup_to = -math.inf
        return None

    def _backup(self):
        """Metres to back up so the side change to the first sign fits (0.55 m + 0.8 m per metre
        of shift, measured with the real servo). The LiDAR cannot see the seat behind the car;
        a sign in the middle seat comes from a one-sign card, so that seat is empty (rules,
        fig. 8c); with a sign at the end seat it may not be, and the car does not back up."""
        if self.pillar is None or self.pillar[1] % 2.0 != 0.5:
            return 0.0
        hold = self.pillar[1] - plan.HOLD
        need = 0.55 + 0.8 * abs(self.offset_at(hold)) - (hold - self.phi)
        return lane.clamp(need, 0.0, BACKUP_LIMIT)

    def _look(self, now, th):
        """Stop short of a sign whose colour is still unknown, back up at full lock so the nose
        (and the camera, 66.7 deg wide) turns toward it, and wait for a frame. In CW the island
        side of the camera only reaches 28.9 deg, so an inner sign just after a corner is
        otherwise never seen before the car must commit to a side."""
        if self._peek is None:
            pillar = self.pillar
            if pillar is None or pillar[2] is not None:
                return None
            seat = int(round(pillar[1] * 2.0)) % 16
            if seat in self._peeked or not PEEK_AT[0] <= pillar[1] - self.phi <= PEEK_AT[1]:
                return None
            self._peeked.add(seat)
            self._peek = [seat, 'back', th, now, 1.0 if pillar[0] > 0.0 else -1.0]
        seat, phase, start_th, since, side = self._peek
        if self.pillar is not None and self.pillar[2] is not None:
            self._peek = None
            return None
        if phase == 'back':
            if abs(lane.wrap(th - start_th)) < PEEK_TURN and now - since < 3.0:
                self.speed = -RECOVER_SPEED
                return -RECOVER_SPEED, -side * self.p.steer_limit * self.sign(), 'RUN'
            self._peek = [seat, 'look', start_th, now, side]
        elif now - since >= PEEK_LOOK:
            self._peek = None
            return None
        self.speed = 0.0
        return 0.0, 0.0, 'RUN'

    def _park(self, now, yaw, gyro, distance):
        """Parking manoeuvres (``park.Parking``); STOP when parked or failed."""
        x, y, th = self._update_pose(distance, yaw, gyro, self.sign())
        speed, steer = self.parking.step(now, x, y, th)
        if self.parking.state in ('PARKED', 'PARK_FAIL'):
            self.state = 'STOP'
            self.speed = 0.0
            return 0.0, 0.0, 'STOP'
        self.speed = speed
        return speed, self.sign() * steer, 'PARK'

    def _update_pose(self, distance, yaw, gyro, sign):
        travelled = distance if self.speed >= 0 else -distance
        self.loc.predict(travelled, sign * lane.wrap(yaw - self.yaw_zero) + self.loc_th0)
        if self._scan is not None and self._scan_stamp > self._last_correct:
            self._last_correct = self._scan_stamp
            bx, by = scan_points(self._scan, sign)
            if bx.size >= 20:
                before = self.loc.th
                self.loc.correct(bx, by, 0.5)
                self.loc_th0 += lane.wrap(self.loc.th - before)
        x, y, th = self.loc.x, self.loc.y, self.loc.th
        self.pose = (x, y, th)
        self.phi = float(lane.unwrap_s(self.phi, lane.track_s(x, y)))
        return x, y, th

    def _speed(self, remaining, offset, front):
        speed = self.p.cruise_speed
        if offset or (self.pillar and self.pillar[1] - self.phi < 0.8):
            speed = min(speed, self.p.pillar_speed)
        if front < 0.7:
            speed = min(speed, 0.2)
        speed = min(speed, math.sqrt(max(0.0, 1.2 * max(remaining, 0.0))))
        if remaining < 1.2:
            speed = min(speed, self.p.finish_speed)
        if speed < 0.02:
            speed = 0.0
        return speed

    def _recover(self, now, front, speed, steer):
        if self._recover_until is not None:
            if now < self._recover_until:
                self.speed = -RECOVER_SPEED
                return -RECOVER_SPEED, steer, 'RUN'
            self._recover_until = None
        if front < 0.16 and speed > 0.0:
            self._recover_until = now + RECOVER_SECONDS
            self.speed = -RECOVER_SPEED
            return -RECOVER_SPEED, steer, 'RUN'
        self.speed, self.steer = speed, steer
        return speed, steer, 'RUN'

    def _walls(self, sign):
        left = _window(self._scan, SIDE_CENTER, SIDE_HALF_WIDTH) if self._scan else np.empty(0)
        right = _window(self._scan, -SIDE_CENTER, SIDE_HALF_WIDTH) if self._scan else np.empty(0)
        left = float(np.median(left)) if left.size else math.inf
        right = float(np.median(right)) if right.size else math.inf
        inner, outer = (left, right) if sign > 0 else (right, left)
        front_window = _window(self._scan, 0.0, FRONT_HALF_WIDTH) if self._scan else np.empty(0)
        front = float(np.min(front_window)) if front_window.size else math.inf
        return inner, outer, front
