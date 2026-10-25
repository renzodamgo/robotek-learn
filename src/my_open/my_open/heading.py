"""Step 2 - The heading: integrate the gyroscope and steer back to a goal heading."""

from robotek_course.minisim import STEER_LIMIT, clamp, wrap

KP = 0.8     # steering (rad) per radian of heading error


class GyroHeading:
    """The car's heading, from the gyroscope alone (the car has no compass)."""

    def __init__(self):
        self.bias = 0.0      # the gyroscope's constant error (rad/s), measured while the car is still
        self.yaw = 0.0       # rad, zero at Start

    def calibrate(self, samples):
        """``samples``: gyroscope readings taken with the car standing still. Their mean is the bias."""
        self.bias = sum(samples) / len(samples)

    def update(self, gyro, dt):
        """Add the rotation of this tick (the reading without the bias) to the heading; returns the heading."""
        self.yaw += (gyro - self.bias) * dt
        return self.yaw


def heading_steer(goal, yaw):
    """Steering angle that brings the heading ``yaw`` towards ``goal``: proportional to the error, within full lock."""
    return clamp(KP * wrap(goal - yaw), -STEER_LIMIT, STEER_LIMIT)
