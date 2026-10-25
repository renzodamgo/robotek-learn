"""Robotek car geometry and sensor mounting; must match robotek_description's URDF.

Measure and update here for the real car; both strategies read these values.

Measured 20-21 oct 2026 with a caliper and the STEP of the sensor plate (Assembly3.stp):
axles are the lines through the Ackermann mounting holes (front) and the far driver holes
(rear); base_link sits midway between them. Wheel 61.0 mm diameter, 14.5 mm wide; track
133.5 mm front (Hiwonder Ackermann axle) and 123.5 mm rear (138.0 outer width).
"""
import math

WHEELBASE = .1174                     # m, caliper hub screw to hub screw (CAD holes say .1203)
STEER_LIMIT = 0.4363323129985824      # rad, 25 deg (rule limit chosen by the team)
# Real car (circles, 8 oct 2026): at 25 deg of servo the wheels reach ~19.7 deg left and
# ~17.6 deg right. base_adapter compensates per side up to this symmetric limit and the
# strategies get it as steer_limit on the car and with real_car:=true.
REAL_STEER_LIMIT = 0.30718            # rad, 17.6 deg of real wheel angle
HALF_LENGTH = .0945                   # m, half of the 189 mm chassis plate (not the LiDAR tab)
HALF_WIDTH = .045                     # m
LIDAR_OFFSET_X = .128                 # m ahead of base_link (scan axis; STEP, 20 oct 2026)
CAMERA_OFFSET_X = .0565               # m ahead of base_link (camera body centre, behind the LiDAR)
CAMERA_PITCH = .4                     # rad, tilted down
CAMERA_HFOV = math.pi/3               # rad, horizontal field of view
# Real camera after re-aiming it, measured 8 oct 2026 with a pillar on each side and the
# LiDAR (measured on the car). Image column -> LiDAR bearing: effective focal 486 px
# (~67 deg; the lens has barrel distortion, ~590 px near the centre) and the camera turned
# 4.4 deg to the left of the LiDAR; pitched ~10 deg down. The simulation keeps the values
# above unless real_car:=true; the strategies get these as parameters on the car.
REAL_CAMERA_HFOV = 1.164              # rad
REAL_CAMERA_PITCH = .17               # rad
REAL_CAMERA_YAW = .077                # rad, + = looks left of the LiDAR's zero
DISTANCE_MODEL_ACCEL = 2.0            # m/s^2, commanded-speed fallback of the LiDAR odometry
# Overall outline for the rules (the parking lot is 1.5 x the length of the robot) and for the
# planners: the LD19 on its plate tab reaches 147.7 mm ahead of base_link, the plate ends 85.2 mm
# behind it (STEP, 21 oct 2026; confirm with the caliper). Width: plate 90 mm, wheels 148 mm outside.
OUTLINE_FRONT = .1477                 # m ahead of base_link (LiDAR tab)
OUTLINE_REAR = .0852                  # m behind base_link (plate end)
OUTLINE_HALF_WIDTH = .0740            # m, half front track .06675 + half wheel width .00725
REAR_AXLE_X = -.0587                  # m from base_link (half the wheelbase)
FRONT_TRACK = .1335                   # m, wheel centre to wheel centre
REAR_TRACK = .1235                    # m
WHEEL_RADIUS = .0305                  # m
WHEEL_WIDTH = .0145                   # m
LENGTH = OUTLINE_FRONT + OUTLINE_REAR   # 0.233 m overall
OUTLINE_HALF_LENGTH = LENGTH/2        # kept for callers that want a symmetric box
