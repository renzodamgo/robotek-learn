"""The steering stopped answering: the mission asks for full lock, the car moves, and the gyroscope sees no turn.

On the car the servo sometimes does not follow a full-lock command, mostly right after a reversal from one lock to the
other (11 nov 2026: Obstacle 23:59 corner 6, 3.6 s of full left at 0.15 m/s and the gyro at -0.07 rad/s against
~0.4 expected, into the wall; Obstacle 23:56 and Open 23:28 the same). The strategy cannot see the servo, but the
gyroscope says whether the wheels turned.

``SteerWatch.update`` is called every control tick with the command just sent (speed, steer), the gyro (bias removed)
and the distance driven in that tick. It reports a fault when, over the last ``WINDOW`` s:

* the command was at full lock (``LOCK`` x the limit), on the same side, for at least ``LAG + WINDOW`` s (the servo
  needs ~0.2-0.3 s to reach the lock: 50 % of the arc's yaw rate 0.20 s after the turn starts, p90 0.30);
* the car moved (``MIN_SPEED`` m/s on average, from the distance, not the command);
* the gyro gave less than ``RATIO`` of the kinematic yaw rate v tan(steer) / L, or turned the other way.

Good corners give 0.7-0.9 of it on the car (11 nov, 23:59 corners 1 and 5) and in the lab (real_v4); the faults
gave -0.2 to 0. After a fault the mission stops for ``HOLD`` s with the wheels centred, then drives on (the command
comes back to the servo from the centre); the watch starts over."""
from collections import deque
import math

LOCK = 0.85          # x steer_limit: counts as full lock
LAG = 0.35           # s at full lock before the window starts (servo travel)
WINDOW = 0.5         # s of gyro compared with the expected turn
MIN_SPEED = 0.08     # m/s the car must move (distance per tick / dt, averaged over the window)
MIN_EXPECTED = 0.15  # rad/s: below this the expected turn is too small to judge
RATIO = 0.3          # gyro / expected below this = the wheels did not turn
HOLD = 0.6           # s stopped with the wheels centred after a fault


class SteerWatch:
    def __init__(self, steer_limit, wheelbase):
        self.limit = steer_limit
        self.wheelbase = wheelbase
        self.samples = deque()        # (now, dt, steer, distance, gyro) since the lock began
        self.lock_since = None
        self.lock_side = 0.0
        self.hold_until = None
        self.faults = 0
        self.last = None              # dict of the last fault, for the log

    def holding(self, now):
        """True while the mission must stay stopped with the wheels centred after a fault."""
        if self.hold_until is not None and now < self.hold_until:
            return True
        self.hold_until = None
        return False

    def update(self, now, dt, speed, steer, gyro, distance):
        """Feed one control tick (the command just decided). Returns True on a new fault."""
        # Forward only: backing up at lock the car turns little and the distance's sign lags the command (lab, 12 nov
        # 2026: two false faults while reversing at 0.08-0.10 m/s); the car's faults were all driving forward.
        side = math.copysign(1.0, steer) if abs(steer) >= LOCK * self.limit and speed > 1e-3 else 0.0
        if side == 0.0 or side != self.lock_side:
            self.samples.clear()
            self.lock_since = now if side else None
            self.lock_side = side
            return False
        self.samples.append((now, dt, steer, distance, gyro))
        while self.samples and self.samples[0][0] < now - WINDOW:
            self.samples.popleft()
        if now - self.lock_since < LAG + WINDOW or not self.samples:
            return False
        span = sum(s[1] for s in self.samples)
        if span < 0.8 * WINDOW:
            return False
        moved = sum(s[3] for s in self.samples)
        speed_real = moved / span
        if speed_real < MIN_SPEED:
            return False
        expected = speed_real * math.tan(sum(s[2] for s in self.samples) / len(self.samples)) / self.wheelbase
        if abs(expected) < MIN_EXPECTED:
            return False
        measured = sum(s[4] * s[1] for s in self.samples) / span
        ratio = measured / expected
        if ratio >= RATIO:
            return False
        self.faults += 1
        self.last = {'event': 'steer_fault', 'gyro': round(measured, 3), 'expected': round(expected, 3),
                     'ratio': round(ratio, 2), 'speed': round(speed_real, 3), 'steer': round(math.degrees(steer), 1),
                     'lock_s': round(now - self.lock_since, 2), 'count': self.faults}
        self.hold_until = now + HOLD
        self.samples.clear()
        self.lock_since, self.lock_side = None, 0.0
        return True
