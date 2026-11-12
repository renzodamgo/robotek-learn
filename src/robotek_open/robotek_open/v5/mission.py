"""Open v5 mission: SETUP -> RUN -> STOP: a heading law on the straights, full lock in the corners.

Inputs every control tick: dt, the latest Scene (a new object per LiDAR stamp), the raw gyro z and the camera's
direction vote (+1 ccw, -1 cw, 0 none). Output: (speed command, steering angle). The yaw is integrated here from the
gyro, and the distance driven from the commanded speed.
"""
import math
from dataclasses import dataclass
from statistics import median

from robotek_common.vehicle import REAL_STEER_LIMIT, WHEELBASE

LAPS = 3
CORNERS = 4 * LAPS

STEER = REAL_STEER_LIMIT
KP = 0.8                    # heading error -> steering
DAMPING = 0.1               # yaw rate -> steering
TURN_TOLERANCE = 0.3        # rad: arc done within this of the goal heading (0.12, 0.4 on the last, before 12 nov 2026)
CONFIRMATIONS = 2           # consecutive scans with the island end inside the look-ahead
ARM_DISTANCE = 0.25         # m driven after a corner before the next one can start
ISLAND_OPEN = 1.0           # m: an island wall farther than this beside the car means it has ended
END_CLEARANCE = 0.08        # m: the look-ahead shrinks in a narrow lane (inner distance minus this)
LATERAL_GAIN, LATERAL_MAX = 0.3, 0.4
BRAKE_GAIN = 2.0            # 1/s: speed above the corner speed grows this much per metre of gap
DIRECTION_TIMEOUT = 15.0    # s without a direction from the camera: stop instead of guessing
DIRECTION_FALLBACK_WAIT = 1.0   # s the camera gets before direction_fallback is taken
CALIBRATE_SECONDS = 0.3     # s still after Start: gyro bias and the start's front range
FINISH_STOP_WAIT = 0.9      # s at rest before DONE
WALL_HEADING_MAX = math.radians(25.0)   # rad: a larger gyro-vs-wall difference is a bad fit, not drift
WRONG_SIDE_OPEN = 1.2       # m: before the first corner, the side away from the island wider than this (outer walls
WRONG_SIDE_SCANS = 3        # are continuous) in this many scans while the island stays beside: the direction is wrong


@dataclass
class Params:
    cruise_speed: float = 1.28
    # m/s in every corner. The car slows down to it on the approach, once the front wall is near, and drives the arc
    # at it. (Before 12 nov 2026 the last corner had its own last_corner_speed; one speed was as good in the lab.)
    corner_speed: float = 1.0
    # rad: the arc ends this short of the 90 deg goal and the straight-line control takes over (keeps inner_gap from
    # the island); higher = a shorter turn.
    turn_tolerance: float = TURN_TOLERANCE
    # Straight line: a heading offset of lateral_kp x (island distance - inner_gap), at most LATERAL_MAX. 0.3 before
    # 11 nov 2026 (5 cm off -> 0.9 deg only: after a corner the car took the whole straight to line up, the coach).
    lateral_kp: float = LATERAL_GAIN
    brake_margin: float = 0.15            # m: the slow-down for a corner ends this far before turn_front_distance;
                                          # higher = brakes earlier (it starts (speed - corner speed) / 2 m before that)
    inner_gap: float = 0.25               # m kept from the island wall
    turn_front_distance: float = 1.19     # m: front wall (along the track) at which a turn may start
    corner_lookahead: float = 0.35        # m: island end this close (or an open side) starts the turn
    wall_heading_gain: float = 0.0        # share of the gyro-vs-island-wall heading difference corrected per scan
                                          # (0 = off; the car needs it: its gyro reads ~6 % short, 0/320 without, lab)
    finish_speed: float = 0.15
    finish_lead: float = 0.08             # m: brake when the front range is this close to the start's
    direction: str = 'auto'               # ccw, cw or auto (camera)
    # auto and the camera still undecided DIRECTION_FALLBACK_WAIT s after the calibration: take direction_fallback
    # (ccw or cw) and let the LiDAR check fix it before corner 1. '' = wait up to DIRECTION_TIMEOUT and stop (direction_fault).
    # 4 nov 2026, real car with the IMX708: both lines equally long from the start (confidence 0.09).
    direction_fallback: str = ''
    laps: int = LAPS                      # 3 in a round; fewer only for tests on the car (e.g. the finish and brake)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    return max(low, min(high, value))


class OpenMission:
    def __init__(self, params=None):
        self.p = params or Params()
        self.corners = 4 * max(1, int(self.p.laps))
        self.state = 'SETUP'
        self.stop_reason = ''
        self.started = False
        self.direction = self.p.direction if self.p.direction in ('ccw', 'cw') else ''
        self.yaw = self.goal = 0.0                 # gyro heading and goal heading
        self.gyro = 0.0                            # yaw rate without the bias (rad/s)
        self.completed = 0                         # corners finished
        self.arc = False
        self.travelled = 0.0                       # m since the last corner (commanded speed x time)
        self._bias = 0.0
        self._calibrated = False
        self._gyro, self._fronts = [], []
        self._start_front = math.inf
        self._time = self._rest = 0.0
        self._hits = 0
        self._wall_scene = None
        self._scene = None
        self._speed, self._front_scene, self._front_moved = 0.0, None, 0.0
        self.front = math.inf                      # front_axis of the last scan minus the distance driven since
        self._braking = False
        self._wrong_side, self._check_scene = 0, None
        self.direction_flipped = False             # the LiDAR overruled the camera's direction before corner 1
        self.direction_fallback_used = False       # the camera did not decide: direction_fallback was taken

    # ----------------------------------------------------------------- API
    def request_start(self):
        self.started = True

    def operator_stop(self):
        self._stop('operator_stop')

    def crash_stop(self):
        self._stop('crash')

    @property
    def label(self):
        """The name the rest of the system reads (telemetry, judge, LEDs, /robotek/state): the v4 names."""
        if self.state == 'STOP':
            return {'done': 'DONE', 'direction_fault': 'DIRECTION_FAULT',
                    'operator_stop': 'OPERATOR_STOP', 'crash': 'CRASH'}[self.stop_reason]
        if self.state == 'SETUP':
            return 'DETECT_DIRECTION' if self._calibrated else 'CALIBRATE' if self.started else 'WAIT_START'
        return 'FINISH' if self.completed >= self.corners else 'TURN' if self.arc else 'FOLLOW'

    @property
    def forward_yaw(self):
        """The track heading in the car's frame: the front wall is measured along it, not along the car."""
        return wrap(self.goal - self.yaw) if self.state == 'RUN' else 0.0

    def _stop(self, reason):
        self.state, self.stop_reason = 'STOP', reason

    # ---------------------------------------------------------------- step
    def step(self, dt, scene, gyro_z, camera=0):
        if self.state == 'SETUP':
            return self._setup(dt, scene, gyro_z, camera)
        if self.state == 'STOP':
            return 0.0, 0.0
        gyro = self.gyro = gyro_z - self._bias
        self.yaw += gyro * dt
        # Distance from the commanded speed (no encoder; the LiDAR odometry gave the same rounds, 12 nov 2026 lab). The
        # front wall between scans is the last scan's minus the distance since (the LD19 skipped 0.2-0.5 s of scans).
        moved = self._speed * dt
        self.travelled += moved
        self._front_moved = 0.0 if scene is not self._front_scene else self._front_moved + moved
        self._front_scene = scene
        self.front = scene.front_axis - self._front_moved
        speed, steer = self._run(dt, scene, gyro)
        self._speed = speed
        return speed, steer

    def _setup(self, dt, scene, gyro_z, camera):
        """Quiet while the gyro bias and the start range are measured, then until the camera gives the direction."""
        if not self.started:
            return 0.0, 0.0
        self._time += dt
        if not self._calibrated and self._time < CALIBRATE_SECONDS:
            self._gyro.append(gyro_z)
            self._fronts.append(scene.front_axis)
        elif not self._calibrated:
            self._calibrate()
        else:
            self._choose_direction(camera)
        return 0.0, 0.0

    def _calibrate(self):
        self._calibrated, self._time = True, 0.0
        self._bias = sum(self._gyro) / len(self._gyro) if self._gyro else 0.0
        finite = [f for f in self._fronts if math.isfinite(f)]
        self._start_front = median(finite) if finite else math.inf

    def _choose_direction(self, camera):
        if not self.direction and camera:
            self.direction = 'ccw' if camera > 0 else 'cw'
        if not self.direction and self.p.direction_fallback in ('ccw', 'cw') \
                and self._time >= DIRECTION_FALLBACK_WAIT:
            self.direction, self.direction_fallback_used = self.p.direction_fallback, True
        if self.direction:
            self.state, self.travelled = 'RUN', 0.0
        elif self._time >= DIRECTION_TIMEOUT:
            self._stop('direction_fault')

    def _run(self, dt, scene, gyro):
        if self.completed == 0 and not self.arc:
            self._check_direction(scene)
        sign = 1.0 if self.direction == 'ccw' else -1.0
        inner, island_open = self.island_side(scene)
        self._observe(scene, inner)
        self._correct_heading(scene)
        if self.completed >= self.corners:
            return self._finish(dt, scene, gyro)
        if self._turn_due(scene, island_open):
            self.goal = wrap(self.goal + sign * math.pi / 2)
            self.arc = True
        if self.arc:
            return self.p.corner_speed, self._arc_steer(sign)
        bias = self._lateral(inner, island_open, sign)
        steer = clamp(KP * wrap(self.goal + bias - self.yaw) - DAMPING * gyro, -STEER, STEER)
        return self._straight_speed(), steer

    def _check_direction(self, scene):
        """Before corner 1: the side that opens is the island's. If it is the other one, the camera got it wrong.

        4 nov 2026, real car: glare hid the blue line, the camera chose cw on a ccw track, and the car drove into the
        wall waiting for an island end on the right while the corner opened on the left.
        """
        if scene is self._check_scene:
            return
        self._check_scene = scene
        inner, _ = self.island_side(scene)
        outer = scene.right if self.direction == 'ccw' else scene.left
        wrong = math.isfinite(outer) and outer > WRONG_SIDE_OPEN and math.isfinite(inner) and inner < ISLAND_OPEN
        self._wrong_side = self._wrong_side + 1 if wrong else 0
        if self._wrong_side >= WRONG_SIDE_SCANS:
            self.direction = 'cw' if self.direction == 'ccw' else 'ccw'
            self.direction_flipped, self._wrong_side, self._hits = True, 0, 0

    def _correct_heading(self, scene):
        """On a straight, pull the gyro yaw towards the heading the island wall shows (once per scan).

        4 nov 2026, real car: each corner turned 4-6 deg more than the gyro measured, and after two laps the car drove
        35-40 deg towards the island while the mission believed it was straight.
        """
        if (self.p.wall_heading_gain <= 0.0 or self.arc or self.travelled < ARM_DISTANCE
                or scene is self._wall_scene or not math.isfinite(scene.wall_heading)):
            return
        self._wall_scene = scene
        difference = wrap(scene.wall_heading - wrap(self.yaw - self.goal))
        if abs(difference) <= WALL_HEADING_MAX:
            self.yaw += self.p.wall_heading_gain * difference

    def island_side(self, scene):
        """Distance to the island wall beside the car, and whether it has ended (open side)."""
        inner = scene.left if self.direction == 'ccw' else scene.right
        return inner, math.isfinite(inner) and inner > ISLAND_OPEN

    def lookahead(self, inner):
        """How far ahead (base_link x) the island end must be seen to count: shorter close to the island (with
        inner_gap 0.25 the uncapped 0.35 turned into the island; 12 nov 2026, lab)."""
        return min(self.p.corner_lookahead, max(0.0, inner - END_CLEARANCE))

    def _observe(self, scene, inner):
        """Count the island-end sightings (one per new scan)."""
        if scene is not self._scene:
            self._scene = scene
            self._hits = self._hits + 1 if 0.0 <= scene.island_end <= self.lookahead(inner) else 0

    def _turn_due(self, scene, island_open):
        """Front wall near enough, and the island ended (open side, or its end confirmed ahead)."""
        return all(self.turn_checks(scene, island_open))

    def turn_checks(self, scene, island_open):
        """The three conditions of a turn, (ARM_DISTANCE driven since the last corner, front wall near, island
        ended); also shown in RViz."""
        return (not self.arc and self.travelled >= ARM_DISTANCE,
                self.front <= self.p.turn_front_distance,
                island_open or self._hits >= CONFIRMATIONS)

    def _lateral(self, inner, island_open, sign):
        """Heading offset that keeps inner_gap from the island wall (none at a corner)."""
        if not math.isfinite(inner) or island_open:
            return 0.0
        return clamp(sign * self.p.lateral_kp * (inner - self.p.inner_gap), -LATERAL_MAX, LATERAL_MAX)

    def _straight_speed(self):
        """Cruise, slowing down to corner_speed as the front wall nears turn_front_distance + brake_margin."""
        gap = self.front - self.p.turn_front_distance - self.p.brake_margin
        return min(self.p.cruise_speed, self.p.corner_speed + BRAKE_GAIN * max(0.0, gap))

    def _arc_steer(self, sign):
        """Full lock until the heading is within turn_tolerance of the goal; the straight-line law finishes the turn.
        (A ramped reference with feed-forward gave the same rounds, 12 nov 2026 lab.)"""
        if abs(wrap(self.goal - self.yaw)) <= self.p.turn_tolerance:
            self.arc, self.travelled = False, 0.0
            self.completed += 1
        return sign * STEER

    def _finish(self, dt, scene, gyro):
        """Back in the start section: drive slowly until the front range matches the one seen at Start."""
        p = self.p
        steer = clamp(KP * wrap(self.goal - self.yaw) - DAMPING * gyro, -STEER, STEER)
        if not self._braking:
            if not math.isfinite(scene.front_axis):
                return 0.0, steer
            if math.isfinite(self._start_front) and scene.front_axis > self._start_front + p.finish_lead:
                return p.finish_speed, steer
            self._braking = True                           # latched: range noise must not restart the motor
        self._rest += dt
        if self._rest >= FINISH_STOP_WAIT:
            self._stop('done')
        return 0.0, 0.0
