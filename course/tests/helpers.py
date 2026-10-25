"""Test benches for the exercises: small drives in the mini simulator (pure Python, no ROS, no Gazebo)."""
import math

from robotek_course.minisim import Field, MiniCar, WHEELBASE, STEER_LIMIT, clamp, real_speed, wrap

KP = 0.8                     # the heading law's gain (step 2)

CORNER_SPEED = 1.28          # m/s commanded, as in the strategy (the car really reaches 0.6)


def full_lock_rate(speed_command=CORNER_SPEED):
    """Yaw rate (rad/s) the car reaches with the wheels at full lock at this commanded speed."""
    return real_speed(speed_command) * math.tan(STEER_LIMIT) / WHEELBASE


def jump_with_damping(goal, ref, yaw, gyro, rate, dt, damping=0.1):
    """For comparison (step 5): NO ramp, the goal heading is the reference at once, and damping against a yaw rate of 0.

    Same signature as an arc controller. Damping with no ramp has no idea how fast the corner should turn, so it also
    brakes the turn the corner needs.
    """
    steer = KP * wrap(goal - yaw) - damping * gyro
    return clamp(steer, -STEER_LIMIT, STEER_LIMIT), goal


def run_arc(arc_fn, sign=1, realistic=True, servo_delay=None, servo_tau=None, rate_factor=1.0, seconds=4.0, dt=0.02):
    """One 90 degree corner (left for sign=+1, right for -1) starting straight at cruising speed.

    ``rate_factor`` is what the strategy BELIEVES about the car: it is told a full-lock yaw rate ``rate_factor`` times
    the true one (0.7: the car actually turns faster than the speed table says).

    Returns a dict: ``overshoot`` (rad past the goal heading), ``t_close`` (s until within 0.12 rad of it),
    ``yaw`` (the heading every tick) and ``steer`` (the wheel angle every tick).
    """
    options = {}
    if servo_delay is not None:
        options['servo_delay'] = servo_delay
    if servo_tau is not None:
        options['servo_tau'] = servo_tau
    car = MiniCar(realistic=realistic, **options)
    car._bias = 0.0                                   # the gyroscope bias is calibrated away by now
    for _ in range(100):                              # reach cruising speed on the straight
        car.step(CORNER_SPEED, 0.0, dt)
    car.x = car.y = car.yaw = car.time = 0.0
    goal, ref = sign * math.pi / 2, 0.0
    rate = (full_lock_rate() if realistic else CORNER_SPEED * math.tan(STEER_LIMIT) / WHEELBASE) * rate_factor
    overshoot, t_close, yaws, steers = 0.0, None, [], []
    for _ in range(int(seconds / dt)):
        steer, ref = arc_fn(goal, ref, car.yaw, car.gyro(), rate, dt)
        car.step(CORNER_SPEED, steer, dt)
        overshoot = max(overshoot, sign * (car.yaw - goal))
        if t_close is None and abs(goal - car.yaw) <= 0.12:
            t_close = car.time
        yaws.append(car.yaw)
        steers.append(car.steer)
    return dict(overshoot=overshoot, t_close=t_close, yaw=yaws, steer=steers)


def drive_straight(calibrated, seconds=10.0, dt=0.02, speed=0.6):
    """Try to drive straight along +x with the estimated heading; returns the car's TRUE heading at the end (step 2)."""
    from my_open.heading import GyroHeading, heading_steer
    car = MiniCar(realistic=True)
    heading = GyroHeading()
    if calibrated:
        heading.calibrate(car.resting_gyro(30))        # 0.6 s standing still
    for _ in range(int(seconds / dt)):
        steer = heading_steer(0.0, heading.update(car.gyro(), dt))
        car.step(speed, steer, dt)
    return car.yaw


def drive_lane(start_gap=0.45, seconds=14.0, dt=0.02, speed=0.6):
    """Drive east along the island's south wall from ``start_gap`` away; returns [(time, gap)] rows (step 4)."""
    from my_open.heading import GyroHeading
    from my_open.lane import lane_steer
    field = Field()
    car = MiniCar(-1.0, -0.50 - start_gap, 0.0, realistic=True)
    heading = GyroHeading()
    heading.calibrate(car.resting_gyro(30))
    rows = []
    for k in range(int(seconds / dt)):
        gap = field.island_gap(car.x, car.y, car.yaw)           # here the gap is read exactly; Gazebo gives you step 3's Scene
        steer = lane_steer(0.0, heading.update(car.gyro(), dt), gap)
        car.step(speed, steer, dt)
        rows.append((k * dt, gap))
    return rows
