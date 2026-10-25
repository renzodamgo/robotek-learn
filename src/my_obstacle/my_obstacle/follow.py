"""Following the line (step 6): curvature ahead, plus lateral and heading feedback beside the car.

The planned line is ``offset(progress)`` metres from the lane centre. A point of it is
``lane_pose(s) + offset(s) * lane_normal(s)``; three such points 0.15 m apart give its heading
and its curvature. The steering law has three parts:

* feed-forward ``atan(WHEELBASE * curvature)``, taken ``PREVIEW`` (0.1 m) ahead because the servo
  and the motor answer ~0.1 s late: the angle the line needs, without waiting for an error;
* ``- LATERAL_GAIN * error``, the distance to the left of the line beside the car;
* ``- kp * heading error - kd * gyro``, against the line's heading beside the car, damped.

A pure pursuit (aim at a point ahead) cut the corners about 6 cm toward the island, and so did
heading feedback against a point ahead: both feedbacks are measured beside the car.
"""
import math

from robotek_common.vehicle import WHEELBASE

from . import lane

PREVIEW = 0.10          # m ahead for the curvature feed-forward (servo and motor lag)
STENCIL = 0.15          # m between the points the line's heading and curvature come from
LATERAL_GAIN = 6.0      # rad of steer per metre off the planned line


def point(offset, progress):
    """Point of the planned line at a progress value; ``offset`` is a function of progress."""
    lx, ly, _ = lane.lane_pose(progress)
    nx, ny = lane.lane_normal(progress)
    d = offset(progress)
    return lx + d * nx, ly + d * ny


def reference(offset, progress):
    """(x, y, heading, curvature) of the planned line at a progress value."""
    x0, y0 = point(offset, lane.advance(progress, -STENCIL))
    x1, y1 = point(offset, progress)
    x2, y2 = point(offset, lane.advance(progress, STENCIL))
    heading = math.atan2(y2 - y0, x2 - x0)
    cross = (x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1)
    sides = math.hypot(x1 - x0, y1 - y0) * math.hypot(x2 - x1, y2 - y1) * math.hypot(x2 - x0, y2 - y0)
    return x1, y1, heading, 2.0 * cross / max(sides, 1e-9)


def steer(pose, phi, gyro, offset, kp, kd, limit):
    """Steering angle (rad, + left, track frame) that follows the planned line."""
    x, y, th = pose
    px, py, heading, _ = reference(offset, phi)
    kappa = reference(offset, lane.advance(phi, PREVIEW))[3]
    error = -(x - px) * math.sin(heading) + (y - py) * math.cos(heading)
    angle = (math.atan(WHEELBASE * kappa) - LATERAL_GAIN * error
             - kp * lane.wrap(th - heading) - kd * gyro)
    return lane.clamp(angle, -limit, limit)
