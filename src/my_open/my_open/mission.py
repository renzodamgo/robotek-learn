"""Step 7 - The mission: three states that put steps 2 to 6 together."""
import math
from statistics import median

from robotek_course.minisim import STEER_LIMIT, WHEELBASE, clamp, real_speed, wrap
from .heading import GyroHeading, heading_steer
from .lane import GAP, lane_steer
from .arc import arc_steer
from .turn_gate import ISLAND_OPEN, TurnGate

LAPS = 3
CORNERS = 4 * LAPS
CRUISE_SPEED = 1.28            # m/s commanded (the car really reaches 0.6)
LAST_CORNER_SPEED = 0.6        # the last corner is slower so the car can stop on the start section
FINISH_SPEED = 0.15
FINISH_LEAD = 0.08             # m: brake when the wall ahead is this close to the distance seen at Start
STOP_WAIT = 0.9                # s at rest before DONE
CALIBRATE_SECONDS = 0.3
DIRECTION_TIMEOUT = 15.0       # s without a direction from the camera: stop instead of guessing
TURN_TOLERANCE = 0.12          # rad: a corner is done within this of the goal heading (0.4 on the last one)
LAST_TOLERANCE = 0.4
BRAKE_GAIN = 2.0               # 1/s: the speed above the last corner's grows this much per metre of gap
BRAKE_MARGIN = 0.15            # m


class MiniMission:
    """SETUP -> RUN -> STOP. Inputs of ``step``: the time step, the latest Scene, the raw gyro reading, the metres driven
    since the last tick and the camera's vote (+1 counter-clockwise, -1 clockwise, 0 unknown). Output: (speed, steer)."""

    def __init__(self, direction='auto'):
        self.state = 'SETUP'
        self.stop_reason = ''
        self.started = False
        self.direction = direction if direction in ('ccw', 'cw') else ''
        self.heading = GyroHeading()
        self.goal = self.ref = 0.0           # the heading to hold, and the ramped reference of a corner
        self.completed = 0                   # corners finished
        self.arc = False
        self.travelled = 0.0                 # m since the last corner
        self.gate = TurnGate()
        self._calibrated = False
        self._samples, self._fronts = [], []
        self._start_front = math.inf
        self._time = self._rest = 0.0
        self._braking = False

    # ----------------------------------------------------------------- API
    def request_start(self):
        self.started = True

    def operator_stop(self):
        self._stop('operator_stop')

    @property
    def yaw(self):
        return self.heading.yaw

    @property
    def label(self):
        """The name the rest of the system reads (telemetry, judge, LEDs)."""
        if self.state == 'STOP':
            return {'done': 'DONE', 'direction_fault': 'DIRECTION_FAULT', 'operator_stop': 'OPERATOR_STOP'}[self.stop_reason]
        if self.state == 'SETUP':
            return 'DETECT_DIRECTION' if self._calibrated else 'CALIBRATE' if self.started else 'WAIT_START'
        return 'FINISH' if self.completed >= CORNERS else 'TURN' if self.arc else 'FOLLOW'

    @property
    def forward_yaw(self):
        """The track heading in the car's frame: the wall ahead is measured along it, not along the car."""
        return wrap(self.goal - self.yaw) if self.state == 'RUN' else 0.0

    def _stop(self, reason):
        self.state, self.stop_reason = 'STOP', reason

    def step(self, dt, scene, gyro_z, travelled, camera=0):
        if self.state == 'SETUP':
            return self._setup(dt, scene, gyro_z, camera)
        if self.state == 'STOP':
            return 0.0, 0.0
        gyro = gyro_z - self.heading.bias
        self.heading.update(gyro_z, dt)
        self.travelled += max(0.0, travelled)
        return self._run(dt, scene, gyro)

    # --------------------------------------------------------------- SETUP
    def _setup(self, dt, scene, gyro_z, camera):
        """Quiet: measure the gyroscope bias and the distance to the wall ahead, then wait for the camera."""
        if not self.started:
            return 0.0, 0.0
        self._time += dt
        if not self._calibrated and self._time < CALIBRATE_SECONDS:
            self._samples.append(gyro_z)
            self._fronts.append(scene.front_axis)
        elif not self._calibrated:
            self.heading.calibrate(self._samples)
            finite = [f for f in self._fronts if math.isfinite(f)]
            self._start_front = median(finite) if finite else math.inf
            self._calibrated, self._time = True, 0.0
        else:
            if not self.direction and camera:
                self.direction = 'ccw' if camera > 0 else 'cw'
            if self.direction:
                self.state, self.travelled = 'RUN', 0.0
            elif self._time >= DIRECTION_TIMEOUT:
                self._stop('direction_fault')
        return 0.0, 0.0

    # ----------------------------------------------------------------- RUN
    def _run(self, dt, scene, gyro):
        sign = 1.0 if self.direction == 'ccw' else -1.0
        inner = scene.left if sign > 0 else scene.right      # the island side
        if self.completed >= CORNERS:
            return self._finish(dt, scene, gyro)
        last = self.completed == CORNERS - 1
        corner_speed = LAST_CORNER_SPEED if last else CRUISE_SPEED
        if self.gate.update(scene, inner, self.travelled, self.arc):
            self.goal = wrap(self.goal + sign * math.pi / 2)
            self.arc = True
            self.gate.corner_started()
        if self.arc:
            return corner_speed, self._corner(dt, gyro, sign, corner_speed)
        island_open = math.isfinite(inner) and inner > ISLAND_OPEN
        keep = GAP if (not math.isfinite(inner) or island_open) else inner      # no lateral offset at a corner
        steer = lane_steer(self.goal, self.yaw, keep, sign)
        return self._straight_speed(scene, last, corner_speed), steer

    def _straight_speed(self, scene, last, corner_speed):
        speed = CRUISE_SPEED
        if last and math.isfinite(scene.front_axis):         # slow down before the last corner
            gap = scene.front_axis - self.gate.turn_front - BRAKE_MARGIN
            speed = min(speed, corner_speed + BRAKE_GAIN * max(0.0, gap))
        return speed

    def _corner(self, dt, gyro, sign, corner_speed):
        rate = real_speed(corner_speed) * math.tan(STEER_LIMIT) / WHEELBASE
        steer, self.ref = arc_steer(self.goal, self.ref, self.yaw, gyro, rate, dt, sign=sign)
        tolerance = LAST_TOLERANCE if self.completed == CORNERS - 1 else TURN_TOLERANCE
        if abs(wrap(self.goal - self.yaw)) <= tolerance:     # the corner is done
            self.arc, self.ref, self.travelled = False, self.goal, 0.0
            self.completed += 1
        return steer

    def _finish(self, dt, scene, gyro):
        """Back at the start: creep until the wall ahead is as near as it was at Start, then stop."""
        steer = clamp(heading_steer(self.goal, self.yaw) - 0.1 * gyro, -STEER_LIMIT, STEER_LIMIT)
        if not self._braking:
            if not math.isfinite(scene.front_axis):
                return 0.0, steer
            if math.isfinite(self._start_front) and scene.front_axis > self._start_front + FINISH_LEAD:
                return FINISH_SPEED, steer
            self._braking = True                             # latched: range noise must not restart the motor
        self._rest += dt
        if self._rest >= STOP_WAIT:
            self._stop('done')
        return 0.0, 0.0
