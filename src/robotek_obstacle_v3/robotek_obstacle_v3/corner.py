"""Line through a corner of the Obstacle v3 lane: straight, one left arc, straight.

Worked in the frame of the first corner (bottom straight -> right straight, island corner at
(0.5, -0.5)); every corner is that one turned, so the result is given in lane coordinates
(progress from the corner start, lateral offset) and applies to all of them.

The line holds offset ``d_a`` on the straight before the corner, turns left on an arc of radius
``r`` and holds ``d_b`` on the straight after it. ``plan`` searches those three numbers for the
largest clearance to the signs at the corner (the last seat of the first straight and the first
seat of the next one), the island and the outer wall, with the car's outline and the side each
sign must be passed on. The result depends only on the signs, so it is cached.

The clearance that decides is the one of the path the car would DRIVE, not of the line: the best
candidates on paper are run through the mission's tracker (feed-forward, lateral and heading
feedback, the real servo's delay, rate and lag, the steering limit) on the line plus the ramps the
mission draws to it from the neighbouring signs' offsets. A 0.2 m ramp into a corner can need
10 1/m of curvature; the car has 2.7, so such a line is clear on paper and touched in the round.
"""
import math
from functools import lru_cache

import numpy as np

from .car import OUTLINE_FRONT, OUTLINE_HALF_WIDTH, OUTLINE_REAR, WHEELBASE

from .settings import Params

try:
    from .corner_table import TABLE
except ImportError:      # before tools/build_corner_table.py has run
    TABLE = {}

SIGN_HALF = 0.025           # m, half of a 50 mm sign
STEP = 0.02                 # m between path samples
RADII = (0.5, 0.6, 0.75)     # m; tighter arcs leave the tracker no steering to correct with
OFFSET_LIMIT = 0.36         # m from the lane centre
OFFSET_GRID = 0.02          # m between candidate offsets
FREE_SHIFT = 0.1            # m an end without a sign may move from the neighbouring sign's offset
ENOUGH = 0.11               # m; past this clearance the line stays near the centre
BEFORE = 0.15               # track units before the corner the plan starts (a sign's hold)
# The mission's tracker (mission._steer), shared so the simulation drives as the car does.
PREVIEW = 0.10              # m ahead for the curvature feed-forward (servo and motor lag)
STENCIL = 0.15              # m between the points the line's heading and curvature come from
LATERAL_GAIN = 6.0          # rad of steer per metre off the planned line
TRACK_SPEED = Params.pillar_speed   # m/s through a corner with signs
TRACK_DT = 0.02             # s, the control period
SERVO = (0.06, 1.2, 0.05)   # s delay, rad/s rate, s lag of the real car's servo (real_car_v1.yaml)
TRACKED = 12                # best candidates on paper that are driven in simulation
RAMP_DS = 0.01              # m between points of the driven reference
HOLD_END = (0.15, -0.35)    # x where the hold before the corner ends: neighbour on the adjacent seat / next one
HOLD_START = (-0.15, 0.35)  # y where the hold after the corner starts: same
LOT_LINE = -0.3             # lane offset of the parking limits' tops (they stand 0.2 m out of the wall)
LOT_SPAN = (0.11, 0.50)     # x of the limits beside the lot before corner 1 (CCW), plates included
AFTER = 0.15                # track units into the next straight it ends at the latest


def _outline():
    """Perimeter points of the car in base_link (x ahead, y left)."""
    xs = np.linspace(-OUTLINE_REAR, OUTLINE_FRONT, 7)
    ys = np.linspace(-OUTLINE_HALF_WIDTH, OUTLINE_HALF_WIDTH, 3)
    points = [(x, y) for x in xs for y in (ys[0], ys[-1])] + [(x, y) for x in (xs[0], xs[-1]) for y in ys[1:-1]]
    return np.array(points)


OUTLINE = _outline()
CORNERS = np.array([(OUTLINE_FRONT, OUTLINE_HALF_WIDTH), (-OUTLINE_REAR, OUTLINE_HALF_WIDTH),
                    (-OUTLINE_REAR, -OUTLINE_HALF_WIDTH), (OUTLINE_FRONT, -OUTLINE_HALF_WIDTH)])


def path(d_a, d_b, r):
    """Samples (x, y, body yaw) of the line in the first corner's frame, or None if it does not fit."""
    y_a, x_b = -1.0 + d_a, 1.0 - d_b
    cx, cy = x_b - r, y_a + r
    x_start = min(0.5 - BEFORE, cx)
    y_end = max(-0.5 + AFTER, cy)
    if x_start < -0.3 or cy > 0.3:
        return None
    # base_link sits half a wheelbase ahead of the rear axle: on an arc its body points
    # slightly outside the tangent.
    slip = math.atan2(WHEELBASE / 2.0, math.sqrt(max(r * r - (WHEELBASE / 2.0) ** 2, 1e-9)))
    line_a = np.arange(x_start, cx, STEP)
    angles = np.arange(-math.pi / 2, 0.0, STEP / r)
    line_b = np.arange(cy, y_end + 1e-9, STEP)
    x = np.concatenate([line_a, cx + r * np.cos(angles), np.full(line_b.size, x_b)])
    y = np.concatenate([np.full(line_a.size, y_a), cy + r * np.sin(angles), line_b])
    yaw = np.concatenate([np.zeros(line_a.size), angles + math.pi / 2 - slip, np.full(line_b.size, math.pi / 2)])
    return x, y, yaw


def _box_distance(px, py, bx, by):
    dx = np.maximum(np.abs(px - bx) - SIGN_HALF, 0.0)
    dy = np.maximum(np.abs(py - by) - SIGN_HALF, 0.0)
    return np.hypot(dx, dy)


def _lot_margin(px, py, lot):
    """Smallest distance of the outline past the line of the parking limits' tops (offset -0.3)
    where they stand: the end of the bottom straight before this corner (CCW, ``lot[0]``) or the
    start of the next straight after it (CW, ``lot[1]``, the mirrored lot)."""
    worst = math.inf
    if lot[0]:
        beside = (px >= LOT_SPAN[0]) & (px <= LOT_SPAN[1])
        if beside.any():
            worst = float((py[beside] + 1.0 - LOT_LINE).min())
    if lot[1]:
        beside = (py >= -LOT_SPAN[1]) & (py <= -LOT_SPAN[0])
        if beside.any():
            worst = min(worst, float((1.0 - LOT_LINE - px[beside]).min()))
    return worst


def clearance(samples, sign_a, sign_b, lot=(False, False), near=(None, None)):
    """Smallest distance (m) of the outline to the signs, the island, the outer wall and (``lot``)
    the parking limits along the samples; negative when a sign is passed on the wrong side.
    ``sign_a``/``sign_b`` are (lateral, colour) or None; ``near`` the signs on the adjacent seats
    outside the corner (the middle seat of the straight before / after it), whose holds the line
    ramps to, or None."""
    x, y, yaw = samples
    c, s = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
    px = x[:, None] + c * OUTLINE[:, 0] - s * OUTLINE[:, 1]
    py = y[:, None] + s * OUTLINE[:, 0] + c * OUTLINE[:, 1]
    qx = x[:, None] + c * CORNERS[:, 0] - s * CORNERS[:, 1]
    qy = y[:, None] + s * CORNERS[:, 0] + c * CORNERS[:, 1]
    island = np.maximum(np.abs(px), np.abs(py)) - 0.5
    outer = 1.5 - np.maximum(np.abs(px), np.abs(py))
    worst = min(float(island.min()), float(outer.min()), _lot_margin(px, py, lot))
    signs = []
    if sign_a is not None:
        # Last seat of the bottom straight: along x, lateral (to the island) along y.
        signs.append((sign_a, 0.5, -1.0 + sign_a[0], qx - 0.5, qy + 1.0))
    if sign_b is not None:
        # First seat of the right straight: along y, lateral along -x.
        signs.append((sign_b, 1.0 - sign_b[0], -0.5, qy + 0.5, 1.0 - qx))
    if near[0]:
        signs.append((near[0], 0.0, -1.0 + near[0][0], qx, qy + 1.0))      # middle seat before
    if near[1]:
        signs.append((near[1], 1.0 - near[1][0], 0.0, qy, 1.0 - qx))      # middle seat after
    for (lateral, colour), bx, by, along, side in signs:
        worst = min(worst, float(_box_distance(px, py, bx, by).min()))
        if colour is None:
            continue
        low, high = _cut(along, side)
        if low.size:
            margin = lateral - SIGN_HALF - high.max() if colour == 'red' else low.min() - lateral - SIGN_HALF
            worst = min(worst, float(margin))
    return worst


def _cut(along, side):
    """Lateral interval where each sample's outline cuts the sign's line (along = 0), as the
    judge measures the side; only samples that straddle the line."""
    a0, a1 = along, np.roll(along, -1, axis=1)
    s0, s1 = side, np.roll(side, -1, axis=1)
    hits = (a0 <= 0.0) != (a1 <= 0.0)
    with np.errstate(divide='ignore', invalid='ignore'):
        cut = np.where(hits, s0 + (s1 - s0) * a0 / (a0 - a1), np.nan)
    rows = hits.any(axis=1)
    return np.nanmin(cut[rows], axis=1), np.nanmax(cut[rows], axis=1)


def _band(sign, fixed):
    """Candidate offsets for the straight next to a corner."""
    if sign is None:
        # No sign at this end of the corner: the line may leave the neighbouring sign's offset.
        return [d for d in np.arange(fixed - FREE_SHIFT, fixed + FREE_SHIFT + 1e-9, 0.05)
                if abs(d) <= 0.3]
    if sign[1] is None:
        return [fixed]
    lateral, colour = sign
    edge = SIGN_HALF + OUTLINE_HALF_WIDTH
    low, high = (-OFFSET_LIMIT, lateral - edge) if colour == 'red' else (lateral + edge, OFFSET_LIMIT)
    return list(np.arange(low, high + 1e-9, OFFSET_GRID)) or [fixed]


def _offsets(sign, fixed, low):
    """Candidate offsets for one end, at or above ``low``."""
    return [d for d in _band(sign, fixed) if d >= low] or [max(fixed, low)]


def _score(gap, d_a, d_b, pref_a, pref_b, r):
    """Enough clearance: stay near the lane centre (shorter shifts, signs stay in the camera's
    view), then prefer a wide arc. Otherwise: the most clearance."""
    if gap >= ENOUGH:
        return (1, -abs(d_a - pref_a) - abs(d_b - pref_b), r)
    return (0, gap, r)


def _ends(spec):
    """(d_a, d_b) of a line: the family's own numbers, or a searched line's first and last offsets."""
    if len(spec) == 3:
        return spec[0], spec[1]
    line = spec[0][1]
    return line[0][1], line[-1][1]


def line_knots(spec):
    """Knots of a corner line: ``(d_a, d_b, r)`` of the straight-arc-straight family, or
    ``(('k', knots),)`` for a line found by tools/build_corner_table.py's search."""
    return spec[0][1] if len(spec) == 1 else knots(*spec)


def _profile(spec, entry, leave):
    """Knots (progress from the corner start, offset) of what the mission asks the car to follow
    through this corner: the neighbouring sign's hold (``entry``/``leave`` = (offset, x or y where
    its hold ends/starts), None when the corner's own sign sets that end), the ramp, the line."""
    line = list(line_knots(spec))
    d_a, d_b = _ends(spec)
    hold_a, at_a = entry or (d_a, line[0][0] + 0.5)
    hold_b, at_b = leave or (d_b, line[-1][0] - 1.5)
    u_a, u_b = min(at_a - 0.5, line[0][0]), max(at_b + 1.5, line[-1][0])
    return [(u_a - 0.3, hold_a), (u_a, hold_a)] + line + [(u_b, hold_b), (u_b + 0.3, hold_b)]


def _lane(u):
    """Lane centre and unit normal (to the island) at progress ``u`` from the corner start."""
    a = (np.clip(u, 0.0, 1.0) - 1.0) * (math.pi / 2)
    cx = np.where(u < 0, 0.5 + u, np.where(u > 1, 1.0, 0.5 + 0.5 * np.cos(a)))
    cy = np.where(u < 0, -1.0, np.where(u > 1, u - 1.5, -0.5 + 0.5 * np.sin(a)))
    nx = np.where(u < 0, 0.0, np.where(u > 1, -1.0, -np.cos(a)))
    ny = np.where(u < 0, 1.0, np.where(u > 1, 0.0, -np.sin(a)))
    return cx, cy, nx, ny


def _advance(u, metres):
    """``scene.advance`` in the corner frame: straights 1 m per unit, the corner 0.785 m."""
    return u + metres / np.where((u >= 0) & (u <= 1), math.pi / 4, 1.0)


def _progress(x, y):
    """The mission's progress of a point: along the straights, the polar angle in the corner."""
    angle = np.arctan2(y + 0.5, x - 0.5)
    return np.where(x <= 0.5, x - 0.5, np.where(y >= -0.5, 1.5 + y, 1.0 + angle / (math.pi / 2)))


class _Profiles:
    """Offset profiles of several candidates on one progress grid, with the mission's reference
    (point, heading, curvature PREVIEW ahead) tabulated once; the tracker only interpolates."""

    def __init__(self, profiles):
        self.u0 = min(k[0][0] for k in profiles)
        grid = np.arange(self.u0, max(k[-1][0] for k in profiles) + RAMP_DS, RAMP_DS)
        self.table = np.stack([np.interp(grid, *zip(*k)) for k in profiles])
        u = np.broadcast_to(grid, self.table.shape)
        self.px, self.py, self.hd, _ = self._reference(u)
        self.kappa = self._reference(_advance(u, PREVIEW))[3]

    def _index(self, u):
        i = np.clip((u - self.u0) / RAMP_DS, 0, self.table.shape[1] - 1.001)
        j = i.astype(int)
        return np.arange(len(self.table)).reshape((-1,) + (1,) * (u.ndim - 1)), j, i - j

    def _point(self, u):
        rows, j, f = self._index(u)
        offset = self.table[rows, j] + (self.table[rows, j + 1] - self.table[rows, j]) * f
        cx, cy, nx, ny = _lane(u)
        return cx + offset * nx, cy + offset * ny

    def _reference(self, u):
        """Point, heading and curvature at ``u``, as ``mission._reference``."""
        x0, y0 = self._point(_advance(u, -STENCIL))
        x1, y1 = self._point(u)
        x2, y2 = self._point(_advance(u, STENCIL))
        cross = (x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1)
        sides = np.hypot(x1 - x0, y1 - y0) * np.hypot(x2 - x1, y2 - y1) * np.hypot(x2 - x0, y2 - y0)
        return x1, y1, np.arctan2(y2 - y0, x2 - x0), 2.0 * cross / np.maximum(sides, 1e-9)

    def at(self, u):
        """Reference point, heading and curvature PREVIEW ahead at each candidate's ``u``."""
        rows, j, f = self._index(u)
        lerp = [t[rows, j] + (t[rows, j + 1] - t[rows, j]) * f for t in (self.px, self.py, self.kappa)]
        h0, h1 = self.hd[rows, j], self.hd[rows, j + 1]
        return lerp[0], lerp[1], h0 + np.arctan2(np.sin(h1 - h0), np.cos(h1 - h0)) * f, lerp[2]


def _command(profiles, x, y, th, steer):
    """The mission's steering command (``mission._steer``) for each candidate."""
    px, py, hd, kappa = profiles.at(_progress(x, y))
    error = -(x - px) * np.sin(hd) + (y - py) * np.cos(hd)
    defaults = Params()
    command = (np.arctan(WHEELBASE * kappa) - LATERAL_GAIN * error
               - defaults.steer_kp * np.arctan2(np.sin(th - hd), np.cos(th - hd))
               - defaults.steer_kd * TRACK_SPEED * np.tan(steer) / WHEELBASE)
    return np.clip(command, -defaults.steer_limit, defaults.steer_limit)


def _drive(profiles, state=None):
    """base_link samples (x, y, yaw) of the tracker on each profile, at TRACK_SPEED with the
    servo's delay, rate and lag; a bicycle about the rear axle (base_link half a wheelbase ahead).
    From the start of each profile on it, or from ``state`` (x, y, heading, steer) for all."""
    plans = _Profiles(profiles)
    count = len(profiles)
    if state is None:
        x, y, th, _ = plans.at(np.array([k[1][0] for k in profiles]))
        steer = np.zeros(count)
    else:
        x, y, th, steer = (np.full(count, float(v)) for v in state)
    rx, ry = x - WHEELBASE / 2 * np.cos(th), y - WHEELBASE / 2 * np.sin(th)
    queue = [steer] * int(round(SERVO[0] / TRACK_DT))
    ends = np.array([k[-1][0] for k in profiles])   # through the hold after the corner
    out, step = [], TRACK_SPEED * TRACK_DT
    while len(out) < 2000:
        x, y = rx + WHEELBASE / 2 * np.cos(th), ry + WHEELBASE / 2 * np.sin(th)
        if (_progress(x, y) >= ends).all():
            break
        out.append((x, y, th))
        queue.append(_command(plans, x, y, th, steer))
        change = (queue.pop(0) - steer) * min(1.0, TRACK_DT / SERVO[2])
        steer = steer + np.clip(change, -SERVO[1] * TRACK_DT, SERVO[1] * TRACK_DT)
        mid = th + 0.5 * step * np.tan(steer) / WHEELBASE
        rx, ry, th = rx + step * np.cos(mid), ry + step * np.sin(mid), th + step * np.tan(steer) / WHEELBASE
    xs, ys, yaws = (np.array(v) for v in zip(*out))
    return [(xs[:, k], ys[:, k], yaws[:, k]) for k in range(len(profiles))]


def _tracked(candidates, sign_a, sign_b, ends, prefs, lot, state=None, near=(None, None)):
    """The candidates (score, clearance, *line) scored by the clearance of the path the tracker
    drives on them; a searched line counts as radius 0.5 in the score."""
    specs = [tuple(c[2:]) for c in candidates]
    out = []
    for spec, driven in zip(specs, _drive([_profile(spec, *ends) for spec in specs], state)):
        gap = clearance(driven, sign_a, sign_b, lot, near)
        r = spec[2] if len(spec) == 3 else RADII[0]
        out.append((_score(gap, *_ends(spec), *prefs, r), gap) + spec)
    return out


def _candidates(sign_a, sign_b, fixed_a, fixed_b, low_a, low_b, prefs):
    """Every line that fits, scored on paper: (score, clearance, d_a, d_b, r)."""
    out = []
    for r in RADII:
        for d_a in _offsets(sign_a, fixed_a, low_a):
            for d_b in _offsets(sign_b, fixed_b, low_b):
                samples = path(d_a, d_b, r)
                if samples is not None:
                    gap = clearance(samples, sign_a, sign_b)
                    out.append((_score(gap, d_a, d_b, *prefs, r), gap, float(d_a), float(d_b), r))
    return out


@lru_cache(maxsize=256)
def plan(sign_a, sign_b, fixed_a, fixed_b, low_a=-1.0, low_b=-1.0, pin_a=False, pin_b=False):
    """Best (clearance, d_a, d_b, r) for a corner. ``fixed_*`` is the offset used on a side
    without a sign of known colour (the neighbouring sign's offset, or the default);
    ``low_*`` the smallest offset allowed on that side (beside the parking lot); ``pin_*``: that
    side has no sign and its neighbouring sign is on the adjacent seat (a short ramp): its
    (lateral, colour), which the driven path must also clear and pass on its side; else False.
    The clearance is the one of the driven path (see the module notes). Looked up in the table of
    tools/build_corner_table.py when it is there (the car does not simulate during the round)."""
    args = (sign_a, sign_b, fixed_a, fixed_b, low_a, low_b, pin_a, pin_b)
    return TABLE[args] if args in TABLE else _choose(args)


def replan(args, state, keep):
    """``plan(*args)`` for a car already in the corner: the candidates, and ``keep`` (the line it
    is on), are driven from ``state`` (x, y, heading, steer in the corner frame), not from the
    ramp before the corner. Not cached: the caller keeps the result."""
    return _choose(args, state, keep)


def context(args):
    """(prefs, entry, leave, lot, near) of a corner's plan arguments (see plan)."""
    sign_a, sign_b, fixed_a, fixed_b, low_a, low_b, pin_a, pin_b = args
    # A free end stays near its neighbour's offset; an end with a sign near the centre.
    prefs = (fixed_a if sign_a is None else 0.0, fixed_b if sign_b is None else 0.0)
    entry = None if sign_a is not None else (fixed_a, HOLD_END[0] if pin_a else HOLD_END[1])
    leave = None if sign_b is not None else (fixed_b, HOLD_START[0] if pin_b else HOLD_START[1])
    return prefs, entry, leave, (low_a > -1.0, low_b > -1.0), (pin_a or None, pin_b or None)


def best(scored):
    """The best of _tracked's rows (a searched line never decides a tie with the family)."""
    return max(scored, key=lambda t: t if len(t) == 5 else t[:2] + (math.inf,))[1:]


def _choose(args, state=None, keep=None):
    sign_a, sign_b = args[:2]
    prefs, entry, leave, lot, near = context(args)
    candidates = sorted(_candidates(*args[:6], prefs), reverse=True)
    if not candidates:
        return None
    shortlist = _shortlist(candidates, entry, leave) + ([(None, None) + tuple(keep)] if keep else [])
    return best(_tracked(shortlist, sign_a, sign_b, (entry, leave), prefs, lot, state, near))


def _shortlist(candidates, entry, leave):
    """The best on paper, and the best that keep the neighbours' offsets (no ramp to drive)."""
    still = [c for c in candidates
             if (entry is None or abs(c[2] - entry[0]) < 1e-6) and (leave is None or abs(c[3] - leave[0]) < 1e-6)]
    return candidates[:TRACKED] + still[:4]


@lru_cache(maxsize=256)
def plan_paper(sign_a, sign_b, fixed_a, fixed_b, low_a=-1.0, low_b=-1.0):
    """The v2 choice: best (clearance, d_a, d_b, r) for a corner on paper (the line, not the driven path). ``fixed_*`` is the offset used on a side
    without a sign of known colour (the neighbouring sign's offset, or the default);
    ``low_*`` the smallest offset allowed on that side (beside the parking lot)."""
    best = None
    # A free end stays near its neighbour's offset; an end with a sign near the centre.
    pref_a = fixed_a if sign_a is None else 0.0
    pref_b = fixed_b if sign_b is None else 0.0
    for r in RADII:
        for d_a in _offsets(sign_a, fixed_a, low_a):
            for d_b in _offsets(sign_b, fixed_b, low_b):
                samples = path(d_a, d_b, r)
                if samples is None:
                    continue
                gap = clearance(samples, sign_a, sign_b)
                score = _score(gap, d_a, d_b, pref_a, pref_b, r)
                if best is None or score > best[0]:
                    best = (score, gap, float(d_a), float(d_b), r)
    if best is None:
        return None
    return best[1:]

@lru_cache(maxsize=256)
def knots(d_a, d_b, r):
    """The line as (progress from the corner start, lateral offset) pairs."""
    x, y, _ = path(d_a, d_b, r)
    return to_knots(x, y)


def to_knots(x, y):
    """Corner-frame points as (progress from the corner start, lateral offset) pairs."""
    out = []
    for px, py in zip(x, y):
        if px <= 0.5:
            out.append((px - 0.5, py + 1.0))
        elif py >= -0.5:
            out.append((1.0 + py + 0.5, 1.0 - px))
        else:
            angle = math.atan2(py + 0.5, px - 0.5)
            out.append((1.0 + angle / (math.pi / 2), 0.5 - math.hypot(px - 0.5, py + 0.5)))
    return tuple(out)
