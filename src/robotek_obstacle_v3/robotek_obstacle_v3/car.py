"""The car geometry Obstacle v3 was tuned with: a copy of WRO-Strategies' robotek_common/vehicle.py (3 nov 2026).

robotek-learn's robotek_common/vehicle.py still has the earlier caliper numbers (21 oct 2026): up to 1.4 mm away
(WHEELBASE .1174 there, .11883 here). corner_table.py was computed with these numbers, so the v3 planners (corner, park, the
mission's outline, the parking search, the LiDAR clusters) read them from here and decide as in WRO-Strategies. Only
drive.py converts the steering to /cmd_vel_raw with robotek_common's WHEELBASE (the one base_adapter inverts).

The original docstring follows.

Robotek car geometry and sensor mounting; must match robotek_description's URDF.

Measure and update here for the real car; both strategies read these values.

From Assembly3.stp (26 oct 2026) and the caliper (20-21 oct 2026). The STEP has the team's sensor
plate with the Hiwonder knuckles, kingpin plate and hex hubs (MentorPi A1 STEP): each kingpin is
coaxial with its Base_v1 hole (73.75 mm apart) and the knuckle sits against the plate; the team's
printed wheel plus silicone (61.0 mm diameter, 14.5 mm wide) slides 7 mm onto the hex. The front
axle is the kingpin line of the STEP, the rear axle the caliper's (123.5 mm track, 138.0 outer);
base_link sits midway between them, on the ground under the tyres. The sensor offsets below are
the earlier STEP values moved with base_link (0.71 mm back, 2.41 mm down).
"""
import math

WHEELBASE = .11883                    # m, front kingpins (STEP) to the rear axle (caliper)
STEER_LIMIT = 0.4363323129985824      # rad, 25 deg (rule limit chosen by the team)
# Real car (circles, 8 oct 2026): at 25 deg of servo the wheels reach ~19.7 deg left and
# ~17.6 deg right. base_adapter compensates per side up to this symmetric limit and the
# strategies get it as steer_limit on the car and with real_car:=true.
REAL_STEER_LIMIT = 0.30718            # rad, 17.6 deg of real wheel angle
HALF_LENGTH = .0945                   # m, half of the 189 mm chassis plate (not the LiDAR tab)
HALF_WIDTH = .045                     # m
LIDAR_OFFSET_X = .12729               # m ahead of base_link (scan axis; STEP)
CAMERA_OFFSET_X = .05579              # m ahead of base_link (camera body centre, behind the LiDAR)
CAMERA_PITCH = .4                     # rad, tilted down
CAMERA_HFOV = math.pi/3               # rad, horizontal field of view
# Real camera after re-aiming it, measured 8 oct 2026 with a pillar on each side and the
# LiDAR (docs/robot-real-pi.md). Image column -> LiDAR bearing: effective focal 486 px
# (~67 deg; the lens has barrel distortion, ~590 px near the centre) and the camera turned
# 4.4 deg to the left of the LiDAR; pitched ~10 deg down. The simulation keeps the values
# above unless real_car:=true; the strategies get these as parameters on the car.
REAL_CAMERA_HFOV = 1.164              # rad
REAL_CAMERA_PITCH = .17               # rad
REAL_CAMERA_YAW = .077                # rad, + = looks left of the LiDAR's zero
REAL_CAMERA_HEIGHT = .14219           # m above the floor (camera board centre, STEP 26 oct 2026)
DISTANCE_MODEL_ACCEL = 2.0            # m/s^2, commanded-speed fallback of the LiDAR odometry
# Overall outline for the rules (the parking lot is 1.5 x the length of the robot) and for the
# planners: the LD19 on its plate tab reaches 147.0 mm ahead of base_link, the plate ends 85.9 mm
# behind it (STEP; confirm with the caliper). Width: plate 90 mm, front wheels 147.6 mm outside.
OUTLINE_FRONT = .14699                # m ahead of base_link (LiDAR tab)
OUTLINE_REAR = .08591                 # m behind base_link (plate end)
OUTLINE_HALF_WIDTH = .0738            # m, half front track .066523 + half wheel width .00725
REAR_AXLE_X = -.059415                # m from base_link (half the wheelbase)
FRONT_TRACK = .13305                  # m, wheel centre to wheel centre (STEP, 26 oct 2026)
KINGPIN_WIDTH = .07375                # m, between the front kingpins (the two Base_v1 holes)
REAR_TRACK = .1235                    # m
WHEEL_RADIUS = .0305                  # m
WHEEL_WIDTH = .0145                   # m
LENGTH = OUTLINE_FRONT + OUTLINE_REAR   # 0.233 m overall
OUTLINE_HALF_LENGTH = LENGTH/2        # kept for callers that want a symmetric box
