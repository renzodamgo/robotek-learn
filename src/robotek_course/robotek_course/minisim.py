"""Mini simulator for the guide: a car, a field and a LiDAR, in plain Python (no ROS, no Gazebo).

Provided to the student; nothing here is an exercise. It mirrors the pieces that matter of the real simulation:

  * the bicycle-model car, with the real servo (dead time 0.06 s, lag 0.05 s, 1.2 rad/s) and the real speed table;
  * the Open Challenge field ``square100`` (outer walls at +-1.50 m, a 1 x 1 m island), the same boxes as the SDF;
  * the LD19 scan of ``real_car:=true``: 306 rays over +-110 deg, 0.12 to 12 m, 8 mm noise and dropouts;
  * a gyroscope with the real noise and bias (0.0019 rad/s noise, 0.006 rad/s bias).

Frame: x forward, y left, z up (as in the URDF). The island is at the centre; the counter-clockwise lap keeps it on the left.
"""
import math
import random
from collections import deque
from dataclasses import dataclass
from types import SimpleNamespace

WHEELBASE = 0.1174           # m (robotek_common/vehicle.py)
STEER_LIMIT = 0.30718        # rad, 17.6 deg: the real car's full lock
LIDAR_X = 0.128              # m ahead of base_link
SCAN_RAYS = 306
SCAN_HALF = 1.91986          # rad, +-110 deg
SCAN_MIN, SCAN_MAX = 0.12, 12.0
SCAN_NOISE, SCAN_DROPOUT = 0.008, 0.043
GYRO_NOISE, GYRO_BIAS = 0.0019, 0.006
SERVO_DELAY, SERVO_TAU, SERVO_RATE = 0.06, 0.05, 1.2
SPEED_TABLE = ((0.0, 0.0), (0.15, 0.18), (0.30, 0.27), (0.40, 0.30), (0.60, 0.42), (1.28, 0.60))
START = (0.0, -1.15)         # the start section, middle of the south corridor

# Outer walls and island, [centre x, centre y, size x, size y] (square100.sdf).
WALLS = (
    (1.52, 0.0, 0.04, 3.08), (-1.52, 0.0, 0.04, 3.08), (0.0, 1.52, 3.08, 0.04), (0.0, -1.52, 3.08, 0.04),
    (0.48, 0.0, 0.04, 1.00), (-0.48, 0.0, 0.04, 1.00), (0.0, 0.48, 1.00, 0.04), (0.0, -0.48, 1.00, 0.04),
)
OUTLINE = (0.1477, 0.0852, 0.0740)   # m: ahead of base_link, behind it, half width


def wrap(angle):
    """Angle in (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    return max(low, min(high, value))


def real_speed(command):
    """Speed the car really reaches for a commanded speed (measured, low battery)."""
    command = abs(command)
    for (x0, y0), (x1, y1) in zip(SPEED_TABLE, SPEED_TABLE[1:]):
        if command <= x1:
            return y0 + (y1 - y0) * (command - x0) / (x1 - x0)
    return SPEED_TABLE[-1][1]


@dataclass(frozen=True)
class Scene:
    """What the LiDAR tells the strategy: four numbers per scan (as in robotek_open/v5/scene.py)."""
    left: float = math.inf          # distance to the wall on the left (m)
    right: float = math.inf
    front_axis: float = math.inf    # distance to the wall ahead along the track
    island_end: float = math.inf    # x of the end of the island wall, in the car's frame (m)


# --------------------------------------------------------------------------- car
class MiniCar:
    """Bicycle-model car. ``realistic=False`` gives an ideal car (instant servo, speed = command).

    ``servo_delay`` and ``servo_tau`` (s) replace the real servo's dead time and lag, to try a slower or faster one.
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, realistic=True, seed=0, servo_delay=SERVO_DELAY, servo_tau=SERVO_TAU):
        self.x, self.y, self.yaw = x, y, yaw
        self.realistic = realistic
        self.servo_delay, self.servo_tau = servo_delay, servo_tau
        self.steer = 0.0                 # wheel angle really reached
        self.speed = 0.0
        self.yaw_rate = 0.0
        self.time = 0.0
        self._queue = None
        self._rng = random.Random(seed)
        self._bias = GYRO_BIAS if realistic else 0.0

    def step(self, speed_command, steer_command, dt):
        """Advance dt seconds with the strategy's command (the same two numbers the real node sends)."""
        steer_command = clamp(steer_command, -STEER_LIMIT, STEER_LIMIT)
        if self.realistic:
            if self._queue is None:
                self._queue = deque([0.0] * max(1, round(self.servo_delay / dt)))
            self._queue.append(steer_command)
            target = self._queue.popleft()                           # dead time
            change = (target - self.steer) * min(1.0, dt / self.servo_tau)  # lag
            self.steer += clamp(change, -SERVO_RATE * dt, SERVO_RATE * dt)
            goal = real_speed(speed_command)
            self.speed += (goal - self.speed) * min(1.0, dt / 0.15)     # the motor takes ~0.15 s
        else:
            self.steer, self.speed = steer_command, speed_command
        self.yaw_rate = self.speed * math.tan(self.steer) / WHEELBASE
        self.yaw = wrap(self.yaw + self.yaw_rate * dt)
        self.x += self.speed * math.cos(self.yaw) * dt
        self.y += self.speed * math.sin(self.yaw) * dt
        self.time += dt

    def gyro(self):
        """What the gyroscope reads: the true yaw rate plus a constant bias and noise."""
        noise = self._rng.gauss(0.0, GYRO_NOISE) if self.realistic else 0.0
        return self.yaw_rate + self._bias + noise

    def resting_gyro(self, n=30):
        """n readings with the car still: what the start calibration averages."""
        return [self._bias + (self._rng.gauss(0.0, GYRO_NOISE) if self.realistic else 0.0) for _ in range(n)]


# ------------------------------------------------------------------------- field
def _slab(o, d, lo, hi):
    """Entry and exit distances of a ray along one axis through [lo, hi], or None when it misses."""
    if abs(d) < 1e-12:
        return (-math.inf, math.inf) if lo <= o <= hi else None
    t1, t2 = (lo - o) / d, (hi - o) / d
    return (t1, t2) if t1 <= t2 else (t2, t1)


class Field:
    """The square100 field: ray casting for the LiDAR and a contact test for the car outline."""

    def __init__(self, walls=WALLS, seed=0):
        self.walls = walls
        self._rng = random.Random(seed)

    def ray_range(self, ox, oy, angle):
        dx, dy = math.cos(angle), math.sin(angle)
        best = math.inf
        for cx, cy, sx, sy in self.walls:
            a, b = _slab(ox, dx, cx - sx / 2, cx + sx / 2), _slab(oy, dy, cy - sy / 2, cy + sy / 2)
            if a is None or b is None:
                continue
            enter, leave = max(a[0], b[0]), min(a[1], b[1])
            if enter <= leave and leave >= 0.0:
                best = min(best, max(enter, 0.0))
        return best

    def scan(self, x, y, yaw, noise=True, dropouts=True):
        """One LD19 scan from the car at (x, y, yaw): an object with the fields of sensor_msgs/LaserScan."""
        ox, oy = x + LIDAR_X * math.cos(yaw), y + LIDAR_X * math.sin(yaw)
        step = 2 * SCAN_HALF / (SCAN_RAYS - 1)
        ranges = []
        for i in range(SCAN_RAYS):
            r = self.ray_range(ox, oy, yaw - SCAN_HALF + i * step)
            if noise:
                r += self._rng.gauss(0.0, SCAN_NOISE)
            if dropouts and self._rng.random() < SCAN_DROPOUT:
                r = math.inf
            ranges.append(r if SCAN_MIN <= r <= SCAN_MAX else math.inf)
        return SimpleNamespace(ranges=ranges, angle_min=-SCAN_HALF, angle_increment=step,
                               range_min=SCAN_MIN, range_max=SCAN_MAX)

    def touches(self, x, y, yaw):
        """True when the car's outline overlaps a wall (separating-axis test, walls are axis-aligned boxes)."""
        front, rear, half = OUTLINE
        c, s = math.cos(yaw), math.sin(yaw)
        corners = [(x + lx * c - ly * s, y + lx * s + ly * c)
                   for lx, ly in ((front, half), (front, -half), (-rear, -half), (-rear, half))]
        for cx, cy, sx, sy in self.walls:
            box = [(cx - sx / 2, cy - sy / 2), (cx + sx / 2, cy - sy / 2), (cx + sx / 2, cy + sy / 2), (cx - sx / 2, cy + sy / 2)]
            axes = [(1.0, 0.0), (0.0, 1.0), (-s, c), (c, s)]
            if all(self._overlap(corners, box, a) for a in axes):
                return True
        return False

    @staticmethod
    def _overlap(a, b, axis):
        pa = [px * axis[0] + py * axis[1] for px, py in a]
        pb = [px * axis[0] + py * axis[1] for px, py in b]
        return max(pa) >= min(pb) and max(pb) >= min(pa)

    def island_gap(self, x, y, yaw):
        """Sideways distance from the car to the island's south wall (counter-clockwise lap, island on the left)."""
        return (-0.50 - y) * math.cos(yaw)
