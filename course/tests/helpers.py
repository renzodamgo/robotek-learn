"""Test benches for the exercises: small drives in the mini simulator (pure Python, no ROS, no Gazebo)."""
import math

from robotek_course.minisim import Field, MiniCar

CORNER_SPEED = 1.0           # m/s commanded in a corner, as in the strategy (the car really reaches 0.53)


def run_arc(arc_fn, sign=1, realistic=True, servo_delay=None, servo_tau=None, seconds=4.0, dt=0.02):
    """One 90 degree corner (left for sign=+1, right for -1) starting straight at the corner speed.

    ``arc_fn(goal, yaw, gyro)`` returns (steering angle, done), like ``my_open.arc.arc_steer``. Once it says done the
    corner does not start again: the bench keeps only its damped heading law (``turn_tolerance=pi``), as the mission does.

    Returns a dict: ``overshoot`` (rad past the goal heading), ``t_close`` (s until within 0.05 rad of it and
    staying there), ``t_lock`` (s at full lock), ``yaw`` (the heading every tick) and ``steer`` (the wheel angle
    every tick).
    """
    options = {}
    if servo_delay is not None:
        options['servo_delay'] = servo_delay
    if servo_tau is not None:
        options['servo_tau'] = servo_tau
    car = MiniCar(realistic=realistic, **options)
    car._bias = 0.0                                   # the gyroscope bias is calibrated away by now
    for _ in range(100):                              # reach the corner speed on the straight
        car.step(CORNER_SPEED, 0.0, dt)
    car.x = car.y = car.yaw = car.time = 0.0
    goal = sign * math.pi / 2
    overshoot, t_lock, yaws, steers, times = 0.0, None, [], [], []
    for _ in range(int(seconds / dt)):
        if t_lock is None:
            steer, done = arc_fn(goal, car.yaw, car.gyro())
            if done:
                t_lock = car.time
        else:                                         # the corner is over: only the damped law from here on, as in the mission
            steer, _ = arc_fn(goal, car.yaw, car.gyro(), turn_tolerance=math.pi)
        car.step(CORNER_SPEED, steer, dt)
        overshoot = max(overshoot, sign * (car.yaw - goal))
        yaws.append(car.yaw)
        steers.append(car.steer)
        times.append(car.time)
    outside = [t for t, yaw in zip(times, yaws) if abs(goal - yaw) > 0.05]
    t_close = outside[-1] + dt if outside and outside[-1] < times[-1] else None
    return dict(overshoot=overshoot, t_close=t_close, t_lock=t_lock, yaw=yaws, steer=steers)


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
