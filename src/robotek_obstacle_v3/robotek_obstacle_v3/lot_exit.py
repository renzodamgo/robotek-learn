"""Start inside the parking lot (``start_lot`` 1, 9 nov 2026): find the direction, leave the lot, then the laps.

The car waits in the lot, parallel to the outer wall and ~1 cm from the rear limit (the team places it so). The
direction comes from the LiDAR's sides: the outer wall is the near side (``direction_in_lot``). Wall on the right ->
counter-clockwise, on the left -> clockwise (the car faces the driving direction).

The exit is the coach's sequence of full-lock pulses (9 nov 2026), ``lot_exit_steps``: space-separated steps
``<side><direction><seconds>``, side L or R at full lock (written for CCW, mirrored for CW: L = away from the wall) or C
with the wheels straight (servo centre, ~1403 us), direction + forward or - reverse. Default ``DEFAULT_STEPS``: L+0.5 R-0.5 L+2.0 R+0.5 R-1.0 L+0.5 (the coach's values at 0.70 m/s after
the first car tries; 0.12-0.25 m/s barely moved the car, a 0.5 s third pulse did not get it out). Then the wheels go
straight and the laps take over (RUN).

Before each pulse the wheels turn with the car stopped (``PRESTEER_S``; the command is a Twist, so a tiny speed under
the motor's deadband, ``PRESTEER_SPEED``, carries the angle), and after it the car stands for ``SETTLE_S``. While
reversing, the rear sensor nearer than ``REAR_STOP_MM`` ends the pulse with a hard brake (``brake``: the node sends
/robotek/estop, which drops the motor to 0 without base_adapter's deceleration ramp).

Why not one arc: in the car's outline model (``parking_search.Lot.free``, lot 0.3675 m, steer 17.5 deg, car 1 cm off the
rear limit and 2 cm off the wall) a forward arc at full lock meets the far limit after ~11 cm, at ~17 deg; out, then
back, then out is what the model needs too. The speed is ``lot_exit_speed``; car.launch (start_lot:=true) gives
base_adapter a kick and pulses below 1.8 rps and no speed ramps, for the LOT_EXIT state only.

The pose estimate for the localizer: the lot from the parameters (``park_fixed.make_lot``), x from the rear sensor,
y from the wall gap, then the pulses as full-lock arcs (``parking_search.move``) that turn the gyro heading.
"""
import math

import numpy as np

from .car import OUTLINE_HALF_WIDTH, OUTLINE_REAR, WHEELBASE
from .parking_search import move

DEFAULT_STEPS = 'L+0.5 R-0.5 L+2.0 R+0.5 R-1.0 L+0.5'
REAR_STOP_MM = 20             # mm of the rear sensor that end the backing move
REAR_MAX_AGE = 0.3            # s; an older rear reading is not used
REAR_SENSOR_X = OUTLINE_REAR + 0.012   # m behind base_link to the sensor's face (car 0.245 m long, not measured)
DEFAULT_REAR_GAP = 0.01       # m to the rear limit when the rear sensor has nothing
SIDE_HALF = math.radians(12.0)  # rad around +-90 deg for the side distances
SETTLE_S = 0.4
PRESTEER_S = 0.5             # s with the car stopped and the wheels turning to full lock before each move
PRESTEER_SPEED = 0.003        # m/s sent meanwhile: the command is a Twist, and at 0 m/s the steering angle cannot be
                              # expressed (base_adapter and the lab keep the last one); under the motor's deadband
MODEL_STEER = 0.8             # x the steer limit for the dead reckoning's arcs: the wheels turn less than asked (servo
                              # lag; the lab's wheelbase): the heading change is taken as a longer path
OUTER = 1.5                   # m from the field centre to the outer wall
WALL_GAP = 0.01               # m between the car's side and the wall it leans on (the team places it so)
WALL_SEEN = 0.13              # m: a wall seen farther than this (the LiDAR's floor 0.12 + 1 cm) is measured


def side_distances(scan):
    """Distance across to the left (+90 deg) and the right (-90 deg) of the LiDAR (median of range x cos of the angle
    off the side, so oblique rays to a wall give the same), nan without returns."""
    ranges = np.asarray(scan.ranges, dtype=float)
    angle = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
    valid = np.isfinite(ranges) & (ranges > max(0.05, scan.range_min)) & (ranges < 4.0)
    out = []
    for center in (math.pi / 2, -math.pi / 2):
        rel = np.arctan2(np.sin(angle - center), np.cos(angle - center))
        pick = valid & (np.abs(rel) <= SIDE_HALF)
        out.append(float(np.median(ranges[pick] * np.cos(rel[pick]))) if pick.any() else math.nan)
    return tuple(out)


def direction_in_lot(scan):
    """(sign, wall distance m): +1 (CCW) with the wall on the right, -1 (CW) on the left; (0, nan) if unsure.

    The LiDAR drops returns nearer than ~0.12 m (scan_adapter's min_valid_range, and the lab's), so the wall the car
    leans on is usually not seen at all: a side with no returns counts as the near one, and its distance is then the
    nominal WALL_GAP + half the car. A side seen farther than WALL_SEEN is used as measured."""
    left, right = side_distances(scan)
    near_left, near_right = not math.isfinite(left), not math.isfinite(right)
    if near_left == near_right:
        if near_left or abs(left - right) < 0.10:
            return 0, math.nan
        near_right = right < left
    side = right if near_right else left
    wall = side if math.isfinite(side) and side > WALL_SEEN else WALL_GAP + OUTLINE_HALF_WIDTH
    return (1, wall) if near_right else (-1, wall)


def parse_steps(text):
    """'L+0.5 R-0.5 ...' -> [(name, direction, side, seconds)]; ValueError on a bad step."""
    steps = []
    for k, word in enumerate(str(text).split()):
        side = {'L': 1.0, 'R': -1.0, 'C': 0.0}.get(word[:1].upper())
        sign = {'+': 1.0, '-': -1.0}.get(word[1:2])
        try:
            seconds = float(word[2:])
        except ValueError:
            seconds = -1.0
        if side is None or sign is None or seconds <= 0:
            raise ValueError("lot_exit_steps: bad step %r (want e.g. 'L+0.5', 'R-1.0' or 'C+0.3')" % word)
        steps.append(('%d_%s%s' % (k + 1, word[0].upper(), 'FWD' if sign > 0 else 'BACK'), sign, side, seconds))
    if not steps:
        raise ValueError('lot_exit_steps: no steps')
    return steps


class LotExit:
    """The pulses out of the lot. ``step`` -> (speed, steer) in the mirrored frame (+ steer = away from the wall)."""

    def __init__(self, lot, wall_distance, rear_mm, speed, steer_limit, steps=DEFAULT_STEPS):
        self.lot, self.speed, self.steer_limit = lot, speed, steer_limit
        self.PULSES = tuple((name, d, side) for name, d, side, _ in parse_steps(steps)) + (('STRAIGHT', 0.0, 0.0),)
        self.lengths = [seconds for *_, seconds in parse_steps(steps)]
        gap = rear_mm / 1000.0 if rear_mm is not None and rear_mm >= 0 else DEFAULT_REAR_GAP
        self.start = (lot.xa + gap + REAR_SENSOR_X, -OUTER + wall_distance, 0.0)
        self.pose = self.start
        self.index = 0
        self.since = None
        self.settle_until = None
        self.heading = 0.0
        self.brake = False           # True for the tick of a hard brake (the node sends /robotek/estop)
        self._last_now = 0.0
        self._last_speed = 0.0       # speed of the last pulse tick (dead reckoning of the C steps)
        self.log = [dict(event='lot_exit_start', pose=[round(v, 3) for v in self.start], rear_mm=rear_mm,
                         steps=str(steps))]

    @property
    def done(self):
        return self.index >= len(self.PULSES)

    @property
    def name(self):
        return 'DONE' if self.done else self.PULSES[self.index][0]

    def step(self, now, heading, rear_mm):
        """heading: gyro heading since the start (mirrored, rad); rear_mm: fresh rear reading or None."""
        self.brake = False
        name, direction, side = self.PULSES[min(self.index, len(self.PULSES) - 1)]
        steer = side * self.steer_limit
        # Dead reckoning: the full-lock arc that turns the heading by the change seen (pulses and rolls).
        dth = heading - self.heading
        model = MODEL_STEER * steer
        curvature = math.tan(model) / WHEELBASE if steer else 0.0
        if abs(dth) > 1e-4 and abs(curvature) > 1e-6:
            self.pose = move(*self.pose, model, dth / curvature)
        elif not steer and self._last_speed:   # wheels straight: the heading says nothing, take the commanded speed
            self.pose = move(*self.pose, 0.0, self._last_speed * max(0.0, now - self._last_now))
        self._last_now = now
        self.heading = heading
        self._last_speed = 0.0
        if self.done:
            return 0.0, 0.0
        if self.settle_until is not None:
            if now < self.settle_until:
                return 0.0, steer
            self.settle_until, self.since = None, None
            self.index += 1
            if self.done:
                return 0.0, 0.0
            name, direction, side = self.PULSES[self.index]
            steer = side * self.steer_limit
        if self.since is None:
            self.since = now
        if name == 'STRAIGHT':       # wheels straight, then done
            if now - self.since < PRESTEER_S:
                return PRESTEER_SPEED, 0.0
            self.log.append(dict(event='lot_exit_done_pulses', heading=round(math.degrees(heading), 1),
                                 pose=[round(v, 3) for v in self.pose]))
            self.index += 1
            return 0.0, 0.0
        if now - self.since < PRESTEER_S:
            return direction * PRESTEER_SPEED, steer
        near = direction < 0 and rear_mm is not None and 0 <= rear_mm < REAR_STOP_MM
        length = self.lengths[self.index]
        if near or now - self.since >= PRESTEER_S + length:
            self.brake = near
            self.log.append(dict(event='lot_exit_pulse', pulse=name, heading=round(math.degrees(heading), 1),
                                 rear_mm=rear_mm, braked=near, pose=[round(v, 3) for v in self.pose]))
            self.settle_until = now + SETTLE_S
            return 0.0, steer
        self._last_speed = direction * self.speed
        return self._last_speed, steer
