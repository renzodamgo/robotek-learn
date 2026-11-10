"""Obstacle v3 mission: SETUP -> RUN -> STOP, lane following plus a side offset per sign.

One observation (``scene.Scene``), one heading law and one stop. The side of a sign is
fixed as soon as its colour is known and held until the car has passed it, instead of
being decided while approaching. No seat map, no beam search, no parking.
"""
import bisect
import math

import numpy as np

from .car import OUTLINE_FRONT, OUTLINE_HALF_WIDTH, OUTLINE_REAR, WHEELBASE
from .field import SWAP, scan_points, track_s, unwrap_s, wrap
from .localizer import Localizer, direction_from_scan
from .perception import scan_clusters
from . import corner, park, park_fixed, scene
from .lot_exit import REAR_MAX_AGE, LotExit, direction_in_lot
from .scene import Scene
from .settings import make

CALIBRATE_SECONDS = 0.3
DIRECTION_MARGIN = (0.10, 0.03)  # inlier-fraction margin to pick the direction: at once, and after
DIRECTION_WAIT = 1.0        # s of scans (the lot limits clutter the outer wall in CCW)
SIDE_CENTER = math.pi / 2
SIDE_HALF_WIDTH = 0.44
FRONT_HALF_WIDTH = 0.35
PREVIEW = 0.10              # m ahead for the curvature feed-forward (servo and motor lag)
STENCIL = 0.15              # m between the points the line's heading and curvature come from
LATERAL_GAIN = 6.0          # rad of steer per metre off the planned line
RECOVER_SPEED = 0.15
RECOVER_SECONDS = 0.8
SECTOR = 0.2                # m from the lane centre a sign still counts (rows at +-0.1; the lot limits reach -0.3)
UNKNOWN_OFFSET = 0.12       # m away from a sign whose colour is still unknown
LOOKOUT = 0.15              # m to the island side to see an inner sign ahead
HOLD = 0.15                 # track units before and after a sign the side offset is held
HORIZON = 3.0               # track units of remembered signs the lateral profile plans over
SEAT_SNAP = 0.15            # track units between a cluster and the seat it is filed under
SEAT_HITS = 2               # sightings before a seat counts as a sign
START_LOOK = (0.25, 1.0)    # s stopped at Start to file the first signs; at most, for a near colour
BACKUP_LIMIT = 0.5          # m the car may back up at Start
GUARD_AHEAD = 0.06          # m of the current arc the contact guard looks along
LOT_OFFSET = -0.19          # m; on the start straight beside the lot the car keeps off its limits
PEEK_AT = (0.6, 1.4)        # track units to an unknown sign where the car stops to look at it
PEEK_TURN = math.radians(15.0)  # rad the nose turns toward the sign, backing up at full lock
PEEK_LOOK = 0.5             # s stopped looking


def _window(scan, center, half):
    """Valid ranges within +-half of a bearing, in the car frame."""
    ranges = np.asarray(scan.ranges, dtype=float)
    if ranges.size == 0:
        return np.empty(0)
    angle = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
    relative = np.arctan2(np.sin(angle - center), np.cos(angle - center))
    valid = np.isfinite(ranges) & (ranges >= max(.16, scan.range_min)) & (ranges <= scan.range_max)
    return ranges[(np.abs(relative) <= half) & valid]


def _neighbour(signs, seats):
    """Side offset of the first remembered sign on those seats (0 without one)."""
    near = [signs[s] for s in seats if s in signs]
    return near[0][2] if near else 0.0


class Mission:
    """The v3 decision chain; the ROS node and the lab only wire sensors to it."""

    def __init__(self, params=None):
        self.p = make(params)
        self.loc = Localizer()
        self.state = 'SETUP'
        self.selected_direction = ''
        self.gyro_bias = 0.0
        self.yaw_zero = 0.0
        self._calib = [0.0, 0.0, 0]
        self._last_now = None
        self._wall_scan = self._direction_scan = -math.inf
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
        self._recover_until = None
        self._recover_steer = None   # steering while backing off after the contact guard
        self._corner_gaps = {}       # corner start -> clearance its line was planned with
        self._lines = {}             # corner start -> (corner.plan arguments, line) last given (tracked_corners)
        self._direction_margin = 0.0
        self._direction_wait = 0.0
        self._seats = {}            # half-unit seat index (0..15) -> [lateral, {colour: votes}, hits]
        self._remembered_scan = -math.inf
        self._run_since = None
        self._backup_to = None      # progress to back up to at Start; -inf once under way
        self._peeked = set()        # seats already looked at
        self._peek = None           # [seat, phase, yaw at start, time]
        self.parking = None
        self._park_heading = None   # gyro heading of the previous parking cycle
        self.lot_exit = None        # LotExit while leaving the lot (start_lot)
        self._rear = (-math.inf, -1)   # (time, mm) of the last rear sensor reading
        if self.p.park and not self.p.park_mode:
            park.nominal_plan(self.p.steer_limit)   # before the round: a few seconds

    # ------------------------------------------------------------- inputs
    def image(self, bgr, stamp):
        """The v3 does not use the field lines; kept so the node keeps one interface."""

    def rear(self, now, mm):
        """Rear sensor reading in mm (-1 = nothing in range)."""
        self._rear = (now, mm)

    def _rear_mm(self, now):
        t, mm = self._rear
        return mm if now - t <= REAR_MAX_AGE and mm >= 0 else None

    def observe(self, observations, stamp=None):
        self._observations = observations or []
        self._obs_stamp = stamp if stamp is not None else math.inf

    def step(self, now, scan, scan_stamp, image_stamp, yaw, gyro, distance):
        dt = 0.0 if self._last_now is None else max(0.0, min(0.1, now - self._last_now))
        self._last_now = now
        gyro -= self.gyro_bias
        new_scan = scan is not None and scan_stamp > self._direction_scan
        if new_scan:
            self._scan = scan
            self._direction_scan = scan_stamp
            self._clusters = scan_clusters(scan)
        if self.state == 'SETUP':
            if self.p.start_lot and self._calib[0] >= CALIBRATE_SECONDS:
                return self._leave_lot(now, scan, yaw)
            return self._setup(dt, scan, new_scan, yaw, gyro)
        if self.state == 'STOP':
            return 0.0, 0.0, 'STOP'
        if self.state == 'PARK':
            return self._park(now, yaw, gyro, distance, scan if new_scan else None)
        return self._run(now, dt, yaw, gyro, distance)

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
                x, y, _ = self.loc.x, self.loc.y, self.loc.th
                self.pose = (x, y, self.loc.th)
                self.phi = float(track_s(x, y))
                self.state = 'RUN'
        return 0.0, 0.0, 'SETUP'

    def _leave_lot(self, now, scan, yaw):
        """start_lot: direction from the LiDAR's sides, the moves out of the lot (lot_exit.py), then RUN."""
        if self.lot_exit is None:
            if scan is None:
                return 0.0, 0.0, 'SETUP'
            sign, wall = direction_in_lot(scan)
            if not sign:
                return 0.0, 0.0, 'SETUP'
            self.selected_direction = 'ccw' if sign > 0 else 'cw'
            self.lot_exit = LotExit(park_fixed.make_lot(sign, self.p), wall, self._rear_mm(now),
                                    self.p.lot_exit_speed, self.p.steer_limit, self.p.lot_exit_pulse,
                                    self.p.lot_exit_back_pulse)
        sign = 1.0 if self.selected_direction == 'ccw' else -1.0
        heading = sign * wrap(yaw - self.yaw_zero)
        speed, steer = self.lot_exit.step(now, heading, self._rear_mm(now))
        if not self.lot_exit.done:
            self.speed = speed
            return speed, sign * steer, 'LOT_EXIT'
        if scan is None:
            return 0.0, 0.0, 'LOT_EXIT'
        x, y, th = self.lot_exit.pose
        self.loc.initialize(*scan_points(scan, sign), th,
                            guesses=[(x + dx, y + dy) for dx in (-0.15, 0.0, 0.15) for dy in (-0.08, 0.0, 0.08)])
        self.loc_th0 = self.loc.th - heading
        self.pose = (self.loc.x, self.loc.y, self.loc.th)
        self.phi = float(track_s(self.loc.x, self.loc.y))
        self.lot_exit.log.append(dict(event='lot_exit_done', estimate=[round(v, 3) for v in (x, y, th)],
                                      localized=[round(v, 3) for v in self.pose]))
        self._backup_to = -math.inf   # no backing up at Start: the lot is behind
        self.state = 'RUN'
        return 0.0, 0.0, 'RUN'

    # ---------------------------------------------------------------- run
    def _run(self, now, dt, yaw, gyro, distance):
        sign = 1.0 if self.selected_direction == 'ccw' else -1.0
        x, y, th = self._update_pose(distance, yaw, gyro, sign)
        _, _, front = self._walls(sign)
        if self._direction_scan > self._remembered_scan:
            self._remembered_scan = self._direction_scan
            self._remember(x, y, th, sign)
        self.pillar = self._next_pillar()
        self.profile = self._plan()
        if self.p.park and self.phi >= self.p.park_after:
            self.state = 'PARK'
            kind = park_fixed.FixedParking if self.p.park_mode else park.Parking
            self.parking = kind(self, self.p.park_speed, self.p.park_margin)
            return 0.0, 0.0, 'PARK'
        remaining = self.p.stop_progress - self.phi
        if remaining <= 0.0 and not self.p.park:
            self.state = 'STOP'
            return 0.0, 0.0, 'STOP'
        if now - self._direction_scan > self.p.scan_max_age:
            self.speed = 0.0
            return 0.0, 0.0, 'RUN'

        start = self._start(now) or self._look(now, th)
        if start is not None:
            return start
        steer = sign * self._steer(x, y, th, gyro)
        speed = self._speed(remaining, self._offset_at(self.phi), front)
        self._guard(now, speed, steer, sign)
        return self._recover(now, front, speed, steer)

    def _guard(self, now, speed, steer, sign):
        """Double-X/STORMS: about to touch -> back off on the other lock (``_recover``), then go on."""
        if self.p.guard_gap > 0.0 and speed > 0.0 and self._recover_until is None and self._touching(sign * steer, sign):
            self._recover_until, self._recover_steer = now + RECOVER_SECONDS, -steer

    def _tight(self):
        """In (or just before) a corner planned with less clearance than tight_gap."""
        if self.p.tight_gap <= 0.0:
            return False
        start = 2 * math.floor((self.phi - 1.0 + corner.BEFORE + 0.2) / 2.0) + 1
        return start - corner.BEFORE - 0.2 <= self.phi <= start + 1.0 + corner.AFTER and \
            self._corner_gaps.get(start, 1.0) < self.p.tight_gap

    def _touching(self, steer, sign):
        """The outline GUARD_AHEAD along the current arc closer than guard_gap to a remembered sign
        or to a wall of the field, from the pose estimate (the LiDAR cannot see the car's sides).
        ``steer`` and the pose are in the mirrored frame."""
        x, y, th = self.pose
        turn = GUARD_AHEAD * math.tan(steer) / WHEELBASE
        fx, fy, fth = x + GUARD_AHEAD * math.cos(th + turn / 2), y + GUARD_AHEAD * math.sin(th + turn / 2), th + turn
        c, s = math.cos(fth), math.sin(fth)
        px = fx + c * corner.OUTLINE[:, 0] - s * corner.OUTLINE[:, 1]
        py = fy + s * corner.OUTLINE[:, 0] + c * corner.OUTLINE[:, 1]
        ring = np.maximum(np.abs(px), np.abs(py))
        gap = min(float((ring - 0.5).min()), float((1.5 - ring).min()))
        for key, (lateral, _, hits) in self._seats.items():
            if hits >= SEAT_HITS:
                lx, ly, _ = scene.lane_pose(key / 2.0)
                nx, ny = scene.lane_normal(key / 2.0)
                gap = min(gap, float(corner._box_distance(px, py, lx + lateral * nx, ly + lateral * ny).min()))
        return gap < self.p.guard_gap
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

    def _look(self, now, th):
        """Stop short of a sign whose colour is still unknown, back up at full lock so the nose
        (and the camera, 66.7 deg wide) turns toward it, and wait for a frame. In CW the
        island side of the camera only reaches 28.9 deg, so an inner sign just after a corner
        is otherwise never seen before the car must commit to a side."""
        if self._peek is None:
            self._peek = self._peek_target(now, th)
            if self._peek is None:
                return None
        seat, phase, start_th, since, side = self._peek
        if self.pillar is not None and self.pillar[2] is not None:
            self._peek = None
            return None
        if phase == 'back':
            if abs(wrap(th - start_th)) < PEEK_TURN and now - since < 3.0:
                self.speed = -RECOVER_SPEED
                return -RECOVER_SPEED, -side * self.p.steer_limit * self._sign(), 'RUN'
            self._peek = [seat, 'look', start_th, now, side]
        elif now - since >= PEEK_LOOK:
            self._peek = None
            return None
        self.speed = 0.0
        return 0.0, 0.0, 'RUN'

    def _peek_target(self, now, th):
        """A new look at the next sign when its colour is unknown and it is PEEK_AT ahead."""
        pillar = self.pillar
        if pillar is None or pillar[2] is not None:
            return None
        seat = int(round(pillar[1] * 2.0)) % 16
        if seat in self._peeked or not PEEK_AT[0] <= pillar[1] - self.phi <= PEEK_AT[1]:
            return None
        self._peeked.add(seat)
        return [seat, 'back', th, now, 1.0 if pillar[0] > 0.0 else -1.0]

    def _sign(self):
        return 1.0 if self.selected_direction == 'ccw' else -1.0

    def _backup(self):
        """Metres to back up so the side change to the first sign fits (0.55 m + 0.8 m per metre
        of shift, measured with the real servo). The LiDAR cannot see the seat behind the car;
        a sign in the middle seat comes from a one-sign card, so that seat is empty (rules,
        fig. 8c); with a sign at the end seat it may not be, and the car does not back up."""
        if self.pillar is None or self.pillar[1] % 2.0 != 0.5:
            return 0.0
        hold = self.pillar[1] - HOLD
        need = 0.55 + 0.8 * abs(self._offset_at(hold)) - (hold - self.phi)
        return scene.clamp(need, 0.0, BACKUP_LIMIT)

    def _park(self, now, yaw, gyro, distance, scan):
        """Parking manoeuvres (``park.Parking``) on the v3 pose; STOP when parked or failed."""
        heading = self._sign() * wrap(yaw - self.yaw_zero)
        turn = wrap(heading - self._park_heading) if self._park_heading is not None else 0.0
        self._park_heading = heading
        x, y, th = self._update_pose(distance, yaw, gyro, self._sign(), self.parking.travelled(distance, turn))
        if scan is not None and hasattr(self.parking, 'observe'):
            self.parking.observe(*scan_points(scan, self._sign()))   # park_fixed_sense: the lot from the LiDAR
        speed, steer = self.parking.step(now, x, y, th)
        if self.parking.state in ('PARKED', 'PARK_FAIL'):
            self.state = 'STOP'
            self.speed = 0.0
            return 0.0, 0.0, 'STOP'
        self.speed = speed
        return speed, self._sign() * steer, 'PARK'

    def _lot_in_laps(self):
        """With lot_laps 0 the laps ignore the lot (as a round without it); parking itself always sees it."""
        return self.p.lot_laps > 0.0 or self.state == 'PARK'

    def _lot_zone(self, progress):
        """Whether a progress value is on the start straight beside the lot (rounds with the lot)."""
        if not self.p.park or not self._lot_in_laps():
            return False
        s = progress % scene.UNITS_PER_LAP
        low, high = (0.43, 1.18) if self.selected_direction == 'ccw' else (-0.18, 0.57)
        return low <= s <= high or low <= s - scene.UNITS_PER_LAP <= high

    def _update_pose(self, distance, yaw, gyro, sign, travelled=None):
        if travelled is None:
            travelled = distance if self.speed >= 0 else -distance
        self.loc.predict(travelled, sign * wrap(yaw - self.yaw_zero) + self.loc_th0)
        if self._scan is not None and self._direction_scan > self._last_correct:
            self._last_correct = self._direction_scan
            bx, by = scan_points(self._scan, sign)
            if bx.size >= 20:
                before = self.loc.th
                self.loc.correct(bx, by, 0.5)
                self.loc_th0 += wrap(self.loc.th - before)
        x, y, th = self.loc.x, self.loc.y, self.loc.th
        self.pose = (x, y, th)
        self.phi = float(unwrap_s(self.phi, track_s(x, y)))
        return x, y, th

    def _steer(self, x, y, th, gyro):
        """Curvature of the planned line a little ahead (feed-forward, for the servo lag) plus
        lateral and heading feedback against the line beside the car; a pure pursuit cut the
        corners ~6 cm toward the island, and so did heading feedback against a point ahead."""
        px, py, heading, _ = self._reference(self.phi)
        preview = max(PREVIEW, abs(self.speed) * self.p.preview_time)
        kappa = self._reference(scene.advance(self.phi, preview))[3]
        error = -(x - px) * math.sin(heading) + (y - py) * math.cos(heading)
        steer = (math.atan(WHEELBASE * kappa) - LATERAL_GAIN * error
                 - self.p.steer_kp * wrap(th - heading) - self.p.steer_kd * gyro)
        return scene.clamp(steer, -self.p.steer_limit, self.p.steer_limit)

    def _point(self, progress):
        lx, ly, _ = scene.lane_pose(progress)
        nx, ny = scene.lane_normal(progress)
        offset = self._offset_at(progress)
        return lx + offset * nx, ly + offset * ny

    def _reference(self, progress):
        """Point, heading and curvature of the planned line at a progress value."""
        x0, y0 = self._point(scene.advance(progress, -STENCIL))
        x1, y1 = self._point(progress)
        x2, y2 = self._point(scene.advance(progress, STENCIL))
        heading = math.atan2(y2 - y0, x2 - x0)
        cross = (x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1)
        sides = math.hypot(x1 - x0, y1 - y0) * math.hypot(x2 - x1, y2 - y1) * math.hypot(x2 - x0, y2 - y0)
        return x1, y1, heading, 2.0 * cross / max(sides, 1e-9)

    def _speed(self, remaining, offset, front):
        # From lap 2 every sign is on the map: the line no longer changes, so it can be driven faster.
        gain = self.p.known_speed_gain if self.phi >= scene.UNITS_PER_LAP else 1.0
        speed = self.p.cruise_speed * gain
        if offset or (self.pillar and self.pillar[1] - self.phi < 0.8):
            speed = min(speed, self.p.pillar_speed * gain)
        if front < self.p.front_distance:
            speed = min(speed, self.p.front_speed)
        if self._tight():
            speed = min(speed, self.p.tight_speed)
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
                return -RECOVER_SPEED, self._recover_steer if self._recover_steer is not None else steer, 'RUN'
            self._recover_until = self._recover_steer = None
        if front < 0.16 and speed > 0.0:
            self._recover_until = now + RECOVER_SECONDS
            self.speed = -RECOVER_SPEED
            return -RECOVER_SPEED, steer, 'RUN'
        self.speed, self.steer = speed, steer
        return speed, steer, 'RUN'

    # ------------------------------------------------------------- helpers
    def _walls(self, sign):
        left = _window(self._scan, SIDE_CENTER, SIDE_HALF_WIDTH) if self._scan else np.empty(0)
        right = _window(self._scan, -SIDE_CENTER, SIDE_HALF_WIDTH) if self._scan else np.empty(0)
        left = float(np.median(left)) if left.size else math.inf
        right = float(np.median(right)) if right.size else math.inf
        inner, outer = (left, right) if sign > 0 else (right, left)
        front_window = _window(self._scan, 0.0, FRONT_HALF_WIDTH) if self._scan else np.empty(0)
        front = float(np.min(front_window)) if front_window.size else math.inf
        return inner, outer, front

    def _remember(self, x, y, th, sign):
        """File every sign cluster under its seat and keep the colours seen for it.

        Signs sit on the straights at three seats (progress 2k, 2k+0.5, 2k+1), 0.4 m or
        0.6 m from the inner wall: lateral +-0.1 m from the lane centre. A seat keeps its
        votes for the whole round, so a colour seen once (even a lap earlier) is not lost
        when the sign leaves the camera's field of view.
        """
        for cluster in self._clusters:
            position = self._pillar_position(cluster, x, y, th, sign)
            if position is None or abs(position[1]) > SECTOR:
                continue
            seat = round(position[0] * 2.0) / 2.0
            if abs(position[0] - seat) > SEAT_SNAP or seat % 2.0 > 1.0:
                continue
            entry = self._seats.setdefault(int(round(seat * 2.0)) % 16, [0.0, {}, 0])
            entry[0] = 0.1 if position[1] >= 0.0 else -0.1
            entry[2] += 1
            colour = self._colour(cluster, sign)
            if colour is not None:
                entry[1][colour] = entry[1].get(colour, 0) + 1

    def _next_pillar(self):
        """Nearest remembered sign ahead: (lateral m, progress units, colour or None)."""
        best = None
        origin = self.phi - 0.05
        for key, (lateral, votes, hits) in self._seats.items():
            if hits < SEAT_HITS:
                continue
            progress = origin + (key / 2.0 - origin) % scene.UNITS_PER_LAP
            if progress - self.phi > self.p.pillar_lookahead:
                continue
            if best is None or progress < best[1]:
                colour = max(votes, key=votes.get) if votes else None
                best = (lateral, progress, colour)
        return best

    def _pillar_position(self, cluster, x, y, th, sign):
        c, s = math.cos(th), math.sin(th)
        lx, ly, lane_heading = scene.lane_pose(self.phi)
        tangent = (math.cos(lane_heading), math.sin(lane_heading))
        cluster_y = sign * cluster['y']
        wx = x + c * cluster['x'] - s * cluster_y
        wy = y + s * cluster['x'] + c * cluster_y
        along = (wx - lx) * tangent[0] + (wy - ly) * tangent[1]
        if not (-0.05 < along < self.p.pillar_lookahead):
            return None
        if math.hypot(cluster['x'], cluster_y) > self.p.pillar_lookahead + 0.5:
            return None
        sign_progress = scene.progress_of(wx, wy, self.phi)
        px, py, _ = scene.lane_pose(sign_progress)
        nx, ny = scene.lane_normal(sign_progress)
        lateral = (wx - px) * nx + (wy - py) * ny
        return sign_progress, lateral

    def _colour(self, cluster, sign):
        if self._obs_stamp < self._direction_scan - 0.6:
            return None
        for observation in self._observations:
            if math.hypot(observation['x'] - cluster['x'], observation['y'] - cluster['y']) < 0.12:
                colour = observation.get('color')
                return colour if sign > 0 else SWAP.get(colour, colour)
        return None

    def _side_offset(self, lateral, colour):
        """Lateral offset that passes a sign on its side; away from it while the colour is unknown."""
        if colour is None:
            # Sign rows are 0.1 m off the centre and the car is 0.148 m wide.
            return -self.p.unknown_offset if lateral >= 0.0 else self.p.unknown_offset
        if self.p.lane_offset > 0.0:
            # Two lanes, as if the sign took both rows of its side.
            return -self.p.lane_offset if colour == 'red' else self.p.lane_offset
        clearance = self.p.pillar_clearance + OUTLINE_HALF_WIDTH
        offset = lateral - clearance if colour == 'red' else lateral + clearance
        return scene.clamp(offset, -0.36, 0.36)

    def _plan(self):
        """Knots of the planned lateral offset. Each remembered sign ahead holds its side
        offset around it, with straight ramps between signs; each corner is replaced by the
        straight-arc-straight line of ``corner.plan`` for the signs at both ends of it."""
        signs = {}
        origin = self.phi - 1.5
        for key, (lateral, votes, hits) in self._seats.items():
            if hits < SEAT_HITS:
                continue
            progress = round(origin + (key / 2.0 - origin) % scene.UNITS_PER_LAP, 2)
            if progress - self.phi <= HORIZON + 1.0:
                colour = max(votes, key=votes.get) if votes else None
                signs[progress] = (lateral, colour, self._side_offset(lateral, colour))
        knots = [(s + d, sign[2]) for s, sign in signs.items() for d in (-HOLD, HOLD)]
        knots += self._lookouts(signs)
        first = 2 * math.floor((self.phi - 1.3 - 1.0) / 2.0) + 1
        for start in range(first, int(self.phi + HORIZON) + 1, 2):
            line = self._corner(start, signs)
            if line:
                knots = [k for k in knots if not line[0][0] <= k[0] <= line[-1][0]] + line
        return sorted(knots)

    def _lookouts(self, signs):
        """Knots that bring a corner-exit sign of unknown colour into the camera's view.

        The camera (66.7 deg) sees a sign at the start of the next straight from the middle of
        the current one only from the island side of the lane (+0.15 m; 0 m for the outer row),
        so the line passes there 1.5 track units before the sign when no other sign is near.
        """
        out = []
        for s, (lateral, colour, _) in signs.items():
            look = s - 1.5
            if colour is not None or round(s) % 2 or look - self.phi < 0.1:
                continue
            if any(abs(other - look) < 0.4 for other in signs):
                continue
            out.append((look, LOOKOUT if lateral > 0.0 else 0.0))
        return out

    def _corner(self, start, signs):
        """(progress, offset) knots of the line through the corner starting at ``start``."""
        sign_a, sign_b = signs.get(float(start)), signs.get(float(start + 1))
        fixed_a = sign_a[2] if sign_a else _neighbour(signs, (start - 0.5, start - 1.0))
        fixed_b = sign_b[2] if sign_b else _neighbour(signs, (start + 1.5, start + 2.0))
        args = (sign_a and sign_a[:2], sign_b and sign_b[:2], round(fixed_a, 2), round(fixed_b, 2),
                *self._lot_limits(start))
        planned = self._tracked(start, signs, args) if self.p.tracked_corners else corner.plan_paper(*args)
        if planned is None:
            return []
        self._corner_gaps[start] = planned[0]
        return [(start + s, d) for s, d in corner.line_knots(planned[1:])]

    def _tracked(self, start, signs, args):
        """corner.plan (the driven path) with the short-ramp pins; replanned from the car inside it."""
        # A free end whose neighbouring sign is on the adjacent seat ramps to the line in ~0.2 m.
        before, after = signs.get(float(start - 0.5)), signs.get(float(start + 1.5))
        pins = (not args[0] and before is not None and before[:2], not args[1] and after is not None and after[:2])
        args = args + pins
        return self._corner_line(start, args, corner.plan(*args))

    def _corner_line(self, start, args, planned):
        """The corner's line; when it changes with the car already in the corner, the one that
        drives best from where the car is (``corner.replan``), kept until the signs change again."""
        if planned is None:
            return None
        last = self._lines.get(start)
        if last and last[0] == args:
            return last[1]
        inside = (self.p.corner_replan and last is not None and start - corner.BEFORE <= self.phi <= start + 1 + corner.AFTER
                  and planned[1:] != last[1][1:])
        if inside:
            planned = corner.replan(args, self._corner_state(start), last[1][1:]) or planned
        self._lines[start] = (args, planned)
        return planned

    def _corner_state(self, start):
        """(x, y, heading, steer) of the car in the frame of the corner starting at ``start``."""
        _, _, h = scene.lane_pose(start)
        x, y, th = self.pose
        c, s = math.cos(h), math.sin(h)
        return (c * x + s * y, -s * x + c * y, wrap(th - h), self._sign() * self.steer)

    def _lot_limits(self, start):
        """Smallest offsets before and after a corner: beside the lot (rounds with it) the line
        keeps off its limits, before corner 1 in CCW and after corner 7 in CW."""
        if not self.p.park or not self._lot_in_laps():
            return -1.0, -1.0
        ccw = self.selected_direction == 'ccw'
        return (LOT_OFFSET if ccw and start % 8 == 1 else -1.0,
                LOT_OFFSET if not ccw and start % 8 == 7 else -1.0)

    def _offset_at(self, progress):
        """Planned lateral offset at a progress value (0 without signs)."""
        offset = self._planned(progress)
        return max(offset, LOT_OFFSET) if self._lot_zone(progress) else offset

    def _planned(self, progress):
        knots = self.profile
        if not knots:
            return 0.0
        i = bisect.bisect_left(knots, (progress, -math.inf))   # first knot at or after progress
        if i == 0:
            return knots[0][1]
        if i == len(knots):
            return knots[-1][1]
        (s0, d0), (s1, d1) = knots[i - 1], knots[i]
        return d0 + (d1 - d0) * (progress - s0) / max(s1 - s0, 1e-9)

    # --------------------------------------------------------------- state
    @property
    def label(self):
        return self.state

    def stopped(self):
        return self.state == 'STOP'
