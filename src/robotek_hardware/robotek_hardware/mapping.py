"""Unit conversions between the strategies' commands and the real car (no ROS).

Strategies speak ROS conventions: /cmd_vel_raw Twist with linear.x in m/s and angular.z
in rad/s for a bicycle model (wheelbase 0.1174 m), +steer = left; /scan with 0 rad
straight ahead and angles growing counter-clockwise.

The car: HiWonder RRC Lite servo commanded in
microseconds (centre 1500 us, 550/29 us per degree on one side and 500/29 on the
other, SERVO_SIGN -1, team convention +delta = right), TB6612 motor in percent, and
an STL-19P lidar whose 0 deg points backwards (front 180, left 240, right 120 deg).
"""
from dataclasses import dataclass
import math


@dataclass
class ServoCalibration:
    center_us: float = 1500.
    gain_pos_us_per_deg: float = 550./29.   # team SERVO_GAIN_POS
    gain_neg_us_per_deg: float = 500./29.   # team SERVO_GAIN_NEG
    sign: float = -1.                       # team SERVO_SIGN
    min_us: float = 1000.
    max_us: float = 2050.
    limit_deg: float = 25.                  # rule-compliant limit used by the strategies


@dataclass
class MotorCalibration:
    pct_per_mps: float = 40.    # TO MEASURE on the field: % of PWM per m/s
    min_pct: float = 12.        # TO MEASURE: least % that moves the car (static friction)
    max_pct: float = 100.
    deadband_mps: float = .005  # |v| below this is a stop


def steer_from_twist(linear, angular, wheelbase, previous):
    """Steering angle (rad, +left) of a bicycle-model Twist; keeps the last one when stopped."""
    if abs(linear) < 1e-3:
        return previous
    return math.atan(angular*wheelbase/linear)


def servo_us(steer_rad, cal: ServoCalibration):
    """Servo pulse for a steering angle (rad, +left), with the team's mapping."""
    steer_deg = max(-cal.limit_deg, min(cal.limit_deg, math.degrees(steer_rad)))
    delta = -steer_deg                 # team convention: +delta = right
    d = delta*cal.sign
    gain = cal.gain_pos_us_per_deg if d >= 0. else cal.gain_neg_us_per_deg
    return max(cal.min_us, min(cal.max_us, cal.center_us+d*gain))


def servo_for_wheel(wheel, servo_pts, left_pts, right_pts, wheel_limit=0.):
    """Servo angle that gives a wheel angle (same unit, +left) on the real car.

    servo_pts -> left_pts / right_pts is the measured wheel angle per side (from 0,
    increasing); the inverse is interpolated piecewise and extrapolated with the last
    slope. wheel is clamped to +-wheel_limit first (0 = no clamp). Without a table the
    wheel angle is returned unchanged.
    """
    if wheel_limit > 0.:
        wheel = max(-wheel_limit, min(wheel_limit, wheel))
    if not servo_pts:
        return wheel
    side = left_pts if wheel >= 0. else right_pts
    w, sign = abs(wheel), math.copysign(1., wheel)
    points = [(0., 0.)]+list(zip(side, servo_pts))      # (wheel, servo)
    for (w0, s0), (w1, s1) in zip(points, points[1:]):
        if w <= w1:
            return sign*(s0+(s1-s0)*(w-w0)/(w1-w0))
    (w0, s0), (w1, s1) = points[-2], points[-1]
    return sign*(s1+(s1-s0)*(w-w1)/(w1-w0))


def rate_limit(current, target, accel, decel, dt):
    """Next speed towards target: at most accel (m/s^2) when speeding up, decel when slowing
    down or reversing (through 0 first). accel/decel <= 0 means no limit."""
    if current*target < 0.:
        target = 0.                      # change of direction: stop first
    slowing = abs(target) < abs(current)
    limit = decel if slowing else accel
    if limit <= 0.:
        return target
    step = limit*dt
    return current+max(-step, min(step, target-current))


def motor_pct(speed_mps, cal: MotorCalibration):
    """Signed motor command in % (negative = reverse)."""
    if abs(speed_mps) < cal.deadband_mps:
        return 0.
    pct = cal.min_pct+cal.pct_per_mps*abs(speed_mps)
    return math.copysign(min(cal.max_pct, pct), speed_mps)


def mask_scan(ranges, angle_min, increment, fov_deg=0., min_valid=0.):
    """Ranges as inf outside +-fov_deg (0 = no limit) or closer than min_valid (0 = off)."""
    out = []
    for i, r in enumerate(ranges):
        a = math.atan2(math.sin(angle_min+i*increment), math.cos(angle_min+i*increment))
        if (fov_deg > 0. and abs(math.degrees(a)) > fov_deg) or (min_valid > 0. and r < min_valid):
            out.append(math.inf)
        else:
            out.append(r)
    return out


def rotate_scan_angles(angle_min, angle_max, offset):
    """Angles of a scan whose zero must be turned by offset (rad) to point forward."""
    return angle_min+offset, angle_max+offset


@dataclass
class LowVoltage:
    """True once `confirm` consecutive valid battery readings (V) are below min_v.

    The board publishes the battery at ~1 Hz and single readings can be off (servo load, board
    glitches), so a single low reading does not trip it. Readings below valid_v (a board glitch, 0 mV) are
    ignored; a reading at or above min_v restarts the count. min_v <= 0 disables it."""
    min_v: float = 6.8
    confirm: int = 5
    valid_v: float = 3.
    count: int = 0

    def update(self, volts):
        if self.min_v <= 0. or volts < self.valid_v:
            return self.count >= self.confirm > 0
        self.count = self.count+1 if volts < self.min_v else 0
        return self.count >= self.confirm


# --- IMU axes -----------------------------------------------------------------------------------
# The strategies read the gyro z as the yaw rate (+ = left) and expect the car frame: x forward,
# y left, z up. The RRC Lite IMU turns with the board, so each car axis is named by the board axis
# (and sign) that points the same way: forward_axis='+y' means "board +y points forward".

def parse_axis(spec):
    """'+y', '-x' or 'z' -> (index, sign)."""
    text = str(spec).strip().lower()
    sign = -1. if text.startswith('-') else 1.
    name = text.lstrip('+-')
    if name not in ('x', 'y', 'z'):
        raise ValueError(f'axis must be x, y or z with an optional sign, got {spec!r}')
    return 'xyz'.index(name), sign


def axes_matrix(forward, left, up):
    """Rows = car x (forward), y (left), z (up) as vectors in board axes. Must be right-handed."""
    rows = []
    for spec in (forward, left, up):
        index, sign = parse_axis(spec)
        row = [0., 0., 0.]
        row[index] = sign
        rows.append(row)
    (a, b, c), (d, e, f), (g, h, i) = rows
    det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g)
    if abs(det - 1.) > 1e-9:
        raise ValueError(f'forward={forward} left={left} up={up} is not a right-handed frame '
                         '(forward x left must be up); swap the sign of one axis')
    return rows


def remap_vector(vector, matrix):
    """Board-frame vector -> car frame."""
    return tuple(sum(m*v for m, v in zip(row, vector)) for row in matrix)


def remap_covariance(covariance, matrix):
    """Row-major 3x3 covariance in the board frame -> car frame (R C R^T)."""
    c = [list(covariance[3*i:3*i + 3]) for i in range(3)]
    rc = [[sum(matrix[i][k]*c[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return [sum(rc[i][k]*matrix[j][k] for k in range(3)) for i in range(3) for j in range(3)]


def _snap(vector):
    index = max(range(3), key=lambda k: abs(vector[k]))
    norm = math.sqrt(sum(v*v for v in vector))
    cosine = abs(vector[index])/norm if norm else 0.
    return ('-' if vector[index] < 0 else '+') + 'xyz'[index], math.degrees(math.acos(min(1., cosine)))


def estimate_axes(rest, push):
    """Board axes of up/forward/left from the mean accelerometer at rest (reads +g upwards) and the
    mean acceleration of a forward push (accelerometer readings, m/s^2, board frame).

    Returns {'up', 'forward', 'left'} with the nearest board axis and 'error_deg' = how far each
    measured direction is from that axis (large means the board is tilted or the push was not
    straight)."""
    rest = [float(v) for v in rest]
    push = [float(v) for v in push]
    up_norm = math.sqrt(sum(v*v for v in rest))
    if up_norm < 1e-6:
        raise ValueError('no gravity at rest')
    up = [v/up_norm for v in rest]
    along = sum(p*u for p, u in zip(push, up))
    forward = [p - along*u for p, u in zip(push, up)]
    if math.sqrt(sum(v*v for v in forward)) < 1e-6:
        raise ValueError('the push has no horizontal component')
    norm = math.sqrt(sum(v*v for v in forward))
    forward = [v/norm for v in forward]
    left = [up[1]*forward[2] - up[2]*forward[1], up[2]*forward[0] - up[0]*forward[2],
            up[0]*forward[1] - up[1]*forward[0]]
    result, errors = {}, {}
    for name, vector in (('up', up), ('forward', forward), ('left', left)):
        result[name], errors[name] = _snap(vector)
    result['error_deg'] = {k: round(v, 1) for k, v in errors.items()}
    return result
