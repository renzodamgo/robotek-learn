"""Step 1 - The car: how the steering angle becomes a turn (the bicycle model)."""
import math

from robotek_course.minisim import WHEELBASE


def yaw_rate(speed, steer):
    """How fast the car rotates (rad/s) when it drives at ``speed`` (m/s) with the front wheels at ``steer`` (rad)."""
    return speed * math.tan(steer) / WHEELBASE


def bicycle_step(x, y, yaw, speed, steer, dt):
    """Advance the car dt seconds: first the heading, then the position along the new heading."""
    yaw = yaw + yaw_rate(speed, steer) * dt
    return x + speed * math.cos(yaw) * dt, y + speed * math.sin(yaw) * dt, yaw


def turn_radius(steer):
    """Radius (m) of the circle the car draws with the wheels fixed at ``steer`` (rad)."""
    return WHEELBASE / math.tan(abs(steer))
