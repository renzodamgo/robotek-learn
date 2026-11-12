"""Step 7 - The mission: three states that put steps 2 to 6 together."""
import math
from statistics import median

from robotek_course.minisim import wrap
from .heading import GyroHeading
from .lane import GAP, lateral_bias
from .arc import arc_steer, damped_steer
from .turn_gate import ARM_DISTANCE, ISLAND_OPEN, StraightDistance, TurnGate

LAPS = 3
CORNERS = 4 * LAPS
CRUISE_SPEED = 1.28            # m/s commanded on the straights (the car really reaches 0.6)
CORNER_SPEED = 1.0             # m/s commanded in every corner: the car slows down to it before the corner
BRAKE_GAIN = 2.0               # 1/s: the speed above CORNER_SPEED grows this much per metre still to go
BRAKE_MARGIN = 0.15            # m: the slow-down is over this far before the corner may start
FINISH_SPEED = 0.15
FINISH_LEAD = 0.08             # m: brake when the wall ahead is this close to the distance seen at Start
FINISH_STRAIGHT = 0.1          # rad: the final brake waits until the car is this straight
STOP_WAIT = 0.9                # s at rest before DONE
CALIBRATE_SECONDS = 0.3
DIRECTION_TIMEOUT = 15.0       # s without a direction from the camera: stop instead of guessing
WALL_HEADING_GAIN = 0.3        # share of the gyro-versus-wall heading difference corrected with every scan
WALL_HEADING_MAX = math.radians(25.0)   # a larger difference is a bad line fit, not gyro drift


class MiniMission:
    """SETUP -> RUN -> STOP. Inputs of ``step``: the time step, the latest Scene, the raw gyro reading and the
    camera's vote (+1 counter-clockwise, -1 clockwise, 0 unknown). Output: (speed, steer)."""

    def __init__(self, direction='auto'):
        self.state = 'SETUP'
        self.stop_reason = ''
        self.started = False
        self.direction = direction if direction in ('ccw', 'cw') else ''
        self.heading = GyroHeading()
        self.goal = 0.0                      # the heading to hold: each corner adds +-90 degrees
        self.completed = 0                   # corners finished
        self.arc = False
        self.straight = StraightDistance()   # m since the last corner
        self.gate = TurnGate()
        self._calibrated = False
        self._samples, self._fronts = [], []
        self._start_front = math.inf
        self._time = self._rest = 0.0
        self._braking = False
        self._wall_scene = None

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

    def step(self, dt, scene, gyro_z, camera=0):
        if self.state == 'SETUP':
            return self._setup(dt, scene, gyro_z, camera)
        if self.state == 'STOP':
            return 0.0, 0.0
        gyro = gyro_z - self.heading.bias
        self.heading.update(gyro_z, dt)
        if not self.arc:
            self.straight.update(scene.front_axis)
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
                self.state = 'RUN'
            elif self._time >= DIRECTION_TIMEOUT:
                self._stop('direction_fault')
        return 0.0, 0.0

    # ----------------------------------------------------------------- RUN
    def _run(self, dt, scene, gyro):
        sign = 1.0 if self.direction == 'ccw' else -1.0
        inner = scene.left if sign > 0 else scene.right      # the island side
        self._correct_heading(scene)
        if self.completed >= CORNERS:
            return self._finish(dt, scene, gyro)
        if self.gate.update(scene, inner, self.straight.travelled, self.arc):
            self.goal = wrap(self.goal + sign * math.pi / 2)
            self.arc = True
        if self.arc:
            steer, done = arc_steer(self.goal, self.yaw, gyro)
            if done:                                         # the straight-line law finishes the turn
                self.arc = False
                self.straight.reset()
                self.completed += 1
            return CORNER_SPEED, steer
        island_open = math.isfinite(inner) and inner > ISLAND_OPEN
        keep = GAP if (not math.isfinite(inner) or island_open) else inner      # no lateral offset at a corner
        steer = damped_steer(self.goal + lateral_bias(keep, sign), self.yaw, gyro)
        return self._straight_speed(scene), steer

    def _straight_speed(self, scene):
        """Cruise, slowing down to CORNER_SPEED as the wall ahead nears the distance where the corner may start."""
        gap = scene.front_axis - self.gate.turn_front - BRAKE_MARGIN
        return min(CRUISE_SPEED, CORNER_SPEED + BRAKE_GAIN * max(0.0, gap))

    def _correct_heading(self, scene):
        """On a straight, pull the gyroscope's heading towards the one the island wall shows (once per scan)."""
        if (self.arc or self.straight.travelled < ARM_DISTANCE or scene is self._wall_scene
                or not math.isfinite(scene.wall_heading)):
            return
        self._wall_scene = scene
        difference = wrap(scene.wall_heading - wrap(self.yaw - self.goal))
        if abs(difference) <= WALL_HEADING_MAX:
            self.heading.yaw += WALL_HEADING_GAIN * difference

    def _finish(self, dt, scene, gyro):
        """Back at the start: creep until the wall ahead is as near as it was at Start, then stop."""
        steer = damped_steer(self.goal, self.yaw, gyro)
        if not self._braking:
            if not math.isfinite(scene.front_axis):
                return 0.0, steer
            turned = abs(wrap(self.goal - self.yaw)) > FINISH_STRAIGHT      # still coming out of the last corner
            if turned or (math.isfinite(self._start_front) and scene.front_axis > self._start_front + FINISH_LEAD):
                return FINISH_SPEED, steer
            self._braking = True                             # latched: range noise must not restart the motor
        self._rest += dt
        if self._rest >= STOP_WAIT:
            self._stop('done')
        return 0.0, 0.0
