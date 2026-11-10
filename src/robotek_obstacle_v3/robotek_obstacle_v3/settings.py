"""Parameters of the Obstacle v3 mission (12 ROS parameters, the rest are constants)."""
from dataclasses import dataclass, fields


@dataclass
class Params:
    cruise_speed: float = 0.28      # m/s on a clear lane (0.35 until 4 nov 2026: with the real car of the lab, --car real_v2, reg1-ccw hit red sign 2 at 8.7 s on every seed; 0.28 gave 10/10, Gazebo car 10/10 too)
    pillar_speed: float = 0.30      # m/s while passing a sign or changing lane
    finish_speed: float = 0.15      # m/s on the last metres and the stop
    pillar_clearance: float = 0.18  # m from the sign box to the car's side
    pillar_lookahead: float = 1.80  # m ahead to start the side change
    steer_kp: float = 1.5           # rad of steer per rad of heading error to the planned line
    steer_kd: float = 0.2           # gyro damping
    steer_limit: float = 0.307      # rad, the real car after per-side compensation (17.6 deg)
    stop_progress: float = 24.7     # track units: 3 laps + 0.7 into the start straight
    park: float = 0.0               # 1: park in the lot after the three laps (rounds with the lot)
    park_after: float = 24.0        # track units driven before parking (3 laps; 0 in parking-only tests)
    lot_laps: float = 1.0           # 1: with park, the laps keep off the lot's limits; 0: the laps as without the lot, the lot only for parking (8 nov 2026, car test)
    park_speed: float = 0.06        # m/s while manoeuvring in the lot
    park_margin: float = 0.015      # m kept from the limits and the wall by the parking search
    park_mode: float = 0.0          # 0: the planned manoeuvre (park.py); 1: the fixed manoeuvre closed on sensors (park_fixed.py)
    park_fixed_ahead: float = 0.145  # m past the lot's far limit (base_link) where the fixed manoeuvre starts reversing
    park_fixed_entry: float = 0.76  # rad of heading that ends the fixed entry arc (gyro)
    park_fixed_margin: float = 0.012  # m the fixed manoeuvre's guard keeps from the limits and the wall
    park_fixed_roll: float = 0.012  # m the car rolls after a stop at first (then the measured roll)
    park_fixed_roll_arc: float = 0.0  # m of arc it rolls after a stop at full lock at first (0 = park_fixed_roll)
    park_fixed_sense: float = 0.0   # 1: the fixed manoeuvre places the lot from the outer wall and limits the LiDAR sees
    park_lot_length: float = 0.3495  # m between the limits' inner faces (rules: 1.5 x 0.233 m car; fixed manoeuvre)
    park_lot_depth: float = 0.20    # m from the outer wall (rules; fixed manoeuvre)
    park_lot_end: float = 0.48      # m along the start straight to the far limit's inner face in CCW (mirrored in CW)
    scan_max_age: float = 0.35      # s; older LiDAR brakes
    imu_max_age: float = 0.20       # s; older gyro brakes
    known_speed_gain: float = 1.0   # cruise and pillar speeds x this from lap 2 on (every sign known)
    lane_offset: float = 0.0        # m; > 0: a sign of known colour is passed at +-this, whatever its row
    front_speed: float = 0.20       # m/s cap while something is closer than front_distance ahead
    front_distance: float = 0.55    # m (v2: 0.70)
    unknown_offset: float = 0.0     # m away from a sign whose colour is still unknown (v2: 0.12; STORMS: the middle)
    tracked_corners: float = 1.0    # 1: corners scored by the tracker driven in simulation (corner.plan)
    corner_replan: float = 1.0      # 1: a corner line that changes with the car in it is driven from the car
    tight_gap: float = 0.07         # m; > 0: a corner planned with less clearance than this is driven at tight_speed
    tight_speed: float = 0.15       # m/s
    preview_time: float = 0.0       # s; > 0: the curvature feed-forward looks speed x this ahead (at least PREVIEW)
    guard_gap: float = 0.0          # m; > 0: LiDAR points this close to the outline GUARD_AHEAD on -> back off
    start_lot: float = 0.0          # 1: the car starts inside the lot and leaves it first (lot_exit.py, 9 nov 2026)
    lot_exit_speed: float = 0.06    # m/s of the lot exit's pulses (the motor creeps with base_adapter's kick and pulses)
    lot_exit_pulse: float = 0.5     # s of each forward pulse of the lot exit
    lot_exit_back_pulse: float = 0.3  # s of its reverse pulse (0.5 s on the car, 9 nov 2026: the rear swung almost into the outer wall)


def validate(params):
    known = {f.name for f in fields(Params)}
    unknown = set(params) - known
    if unknown:
        raise ValueError('Unsupported Obstacle v3 parameters: ' + ', '.join(sorted(unknown)))
    return {k: float(v) for k, v in params.items()}


def make(params=None):
    values = validate(params or {})
    return Params(**values)
