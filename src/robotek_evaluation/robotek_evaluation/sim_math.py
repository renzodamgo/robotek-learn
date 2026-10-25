"""Pure maths of the actuator model: no ROS, so the Open lab and the tests run without it."""
import math


def slew(current, target, accel, decel, dt):
    """Move towards target at most accel*dt when speeding up, decel*dt otherwise."""
    limit = (accel if abs(target) > abs(current) else decel)*dt
    return current+max(-limit, min(limit, target-current))


def table_speed(v, cmd, real):
    """Real speed for a commanded one: piecewise-linear table, odd, last slope beyond."""
    if not cmd:
        return v
    x, sign = abs(v), math.copysign(1., v)
    points = [(0., 0.)]+list(zip(cmd, real))
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        if x <= x1:
            return sign*(y0+(y1-y0)*(x-x0)/(x1-x0))
    (x0, y0), (x1, y1) = points[-2], points[-1]
    return sign*(y1+(y1-y0)*(x-x1)/(x1-x0))


def wheel_angle(servo, deadband, gain):
    """Effective wheel angle for a servo angle with play (deadband) and a gain."""
    if deadband <= 0. and gain == 1.:
        return servo
    return math.copysign(max(0., abs(servo)-deadband)*gain, servo)
