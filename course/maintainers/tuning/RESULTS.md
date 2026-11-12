# Tuning results (chapter 12, "Afinar")

Raw material for `Portfolio-Astro/src/content/learn/steps/open-strategy/paso-12-afinar.md`. Everything here was measured
on 12 Nov 2026 with the reference strategy `src/my_open` (the course solution after the rework of that day: full lock
in the corners until `TURN_TOLERANCE` from the goal, one `CORNER_SPEED` with a slow-down before every corner, the
distance driven from the wall ahead, the island-wall heading correction) in the **Open lab**
(`robotek_evaluation.open_lab`, no Gazebo) unless it says Gazebo. Nothing was run on the real car. The raw tables are
in `raw/*.md`, the per-round data in `raw/*.json`; they are appended at the end of this file unchanged. The tables of
the strategy before the rework (ramp, feed-forward, `LAST_CORNER_SPEED`) are in the git history of this file.

## Setup

* Suite: the standard 12 rounds (6 fields x 2 directions), seed 0 unless a table says otherwise. Sweeps that need to
  tell a small difference from noise use seeds 0,1,2 (36 rounds per row).
* Columns of every table: `ok` valid rounds / rounds; `pts` mean points; `t` mean time from Start of the 30-point
  rounds; `sq100` time of the `square100` field, ccw (the Gazebo field); `clr` smallest clearance of the car outline to
  a wall in the whole suite, in cm; `failures` how the bad rounds ended. A round is valid with 30 points, no contact,
  stop observed, margin >= 1 cm and steering <= 25.5 degrees (docstring of `lab_sweep.py`). A row with `pts 30.0`
  but fewer valid rounds and no entry under `failures` (for example `wheelbase 14.7 cm`, `delay x3, DAMPING 0.2`) has
  rounds that were invalid for a margin below 1 cm without touching a wall: this is an inference from the columns, the
  json has the per-round reason (`failed`).
* The car is `real_car_v1.yaml` (servo delay 0.06 s, lag 0.05 s, rate 1.2 rad/s, steering limit 0.30718 rad = 17.6
  degrees, speed table commanded 1.28 -> real 0.60 m/s, which is an inferred point, not a measurement: see the
  comment in the yaml). In the lab the servo is modelled as Gazebo shows it: `actuator_parameters()` replaces
  `steer_delay` by 0.10 s and `steer_tau` by 0.06 s (the yaml value plus what the Gazebo plugin adds). So a row
  "delay x2 = 0.12 s" in a lab table means `--act steer_delay=0.16 --act steer_tau=0.06 --act steer_rate=1.2` in the lab,
  and `actuator_params:=steer_delay=0.12` in Gazebo (the plugin adds its 0.04 s there).
* Baseline: 12/12 valid, mean time 41.82 s, 33.78 s on square100, smallest clearance 10.1 cm (seed 0). With seeds
  0,1,2: 36/36, 41.85 s, 33.78 s, 9.9 cm. Gazebo (square100, no window, `gazebo.tsv`): ccw 30 of 30 points, 35.56 s,
  laps 11.4 / 11.0 / 11.0 s, margin 17.7 cm; cw 30 of 30, 35.53 s, laps 11.4 / 11.0 / 11.0 s, margin 18.0 cm.
  The lab is about 1.8 s faster than Gazebo on square100 ccw (33.78 vs 35.56).

## Findings

1. **The stop in the start section is the fragile part.** Every failure that is not a crash is a round of 27 points:
   three laps and a stop outside the section. It happens when the car leaves the last corner faster than the
   baseline: `CORNER_SPEED` 1.15 (7/12) or 1.28 (6/12), motor x1.15 (29/36 with three seeds), x1.3 (21/36),
   `drive_tau` 0.3 s (6/12), `FINISH_SPEED` 0.8 (0/12), `FINISH_LEAD` 0.15 (8/12). Lowering `CORNER_SPEED` to 0.9
   brings motor x1.15 back to 36/36 (38.27 s) and costs 1.4 s with the nominal motor (43.28 s, 36/36).
2. **Almost no constant buys time.** Cruise 1.6 / 2.0: -0.3 / -0.5 s (the motor is at its limit); `FINISH_SPEED`
   0.6: -0.3 s; `BRAKE_MARGIN` 0: -0.3 s. Slower costs: cruise 1.0 +1.7 s, corner speed 0.8 +3.0 s, 0.6 +7.1 s.
3. **A slower servo is tolerated up to delay x3** (12/12, clearance 2.9 cm); x5 fails (6/12). Rate 0.6 rad/s leaves
   2.9 cm, 0.4 fails. With delay x3 neither `DAMPING` (0 to 0.3) nor `TURN_TOLERANCE` (0.2 to 0.5) changes the round;
   a lower corner speed gives the clearance back (0.8: 6.1 cm; 0.6: 10.7 cm).
4. **Steering angle.** The car turning more than the strategy assumes changes almost nothing. Strategy and car at
   20 deg: -0.6 s, clearance 5.8 cm; at 23 deg 9/12 and at 25 deg 6/12 (the car turns tighter and overshoots with the
   same `TURN_TOLERANCE`). A car that only reaches 14 deg: 11/12; 11.5 deg: 6/12.
5. **The island-wall heading correction is what makes a real gyroscope usable.** Without it (`WALL_HEADING_GAIN` 0)
   a scale error of 3 % already fails almost every round and 6 % short (the team's car) gives 0/36; with it +-5 % is
   12/12, 6 % short 34/36 (36/36 with `CORNER_SPEED` 0.9) and 10 % short 9/12. Drift up to 0.0005 rad/s per s: 12/12
   with it, 4/12 without. With an exact gyroscope the gain (0 to 1) changes nothing.
6. **Lost LiDAR beams no longer matter** (up to 40 %: 12/12): beams with no return do not end the island wall.
   Range noise of 20 mm fails (10/12).
7. **Constants that hardly change anything:** `GAP` 0.20 to 0.40, `GAIN` 0 to 2, `LIMIT`, `DAMPING` 0 to 0.2,
   `TURN_TOLERANCE` 0.3 to 0.8, `TURN_FRONT` 1.1 to 1.3, `ARM_DISTANCE` 0 to 0.5, `KP` 0.2 to 1.2 (0.4: 11/12).
   **Constants that cost:** `TURN_FRONT` 0.9 (+1.4 s), `LOOKAHEAD` 0.1 (+1.5 s) or 0.5 (3/12 hit the island),
   `TURN_TOLERANCE` 0.12 / 0.2 (11/12, 10/12: stop outside), `GAP` 0.15 (10/12), `KP` 3 (2/12), `ARM_DISTANCE` 1.2
   (0/12).
8. **Others.** Camera: each second of delay is 0.7 to 1 s of round; 16 s is `DIRECTION_FAULT`. Calibration: 0.05 s
   is as good as 0.3 s even with gyro noise 0.01 (the wall correction hides it); longer only costs time. Control
   latency 100 ms: 12/12 with 4.9 cm; 200 ms fails (5/12). Wheelbase 14.7 cm: 9 or 10 of 12.

## Reproduce

    source tools/macos_env.sh; source install/setup.zsh
    python3 course/maintainers/tuning/lab_sweep.py all --seeds 0
    python3 course/maintainers/tuning/lab_sweep.py baseline candidates candidates_robust --seeds 0,1,2
    bash course/maintainers/tuning/gazebo_round.sh base ccw      # and cw

# Appendix: the raw tables, unchanged (`raw/*.md`)

### Baseline

Seeds 0,1,2, 36 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| baseline | 36/36 | 30.0 | 41.85 | 33.78 | 9.9 |  |

### Servo dead time (yaml steer_delay; x1 = 0.06 s)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| delay x0.5 = 0.03 s | 12/12 | 30.0 | 41.43 | 33.28 | 11.0 |  |
| delay x1 = 0.06 s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| delay x1.5 = 0.09 s | 12/12 | 30.0 | 42.26 | 34.18 | 8.5 |  |
| delay x2 = 0.12 s | 12/12 | 30.0 | 42.51 | 34.38 | 6.9 |  |
| delay x3 = 0.18 s | 12/12 | 30.0 | 43.40 | 35.38 | 2.9 |  |
| delay x5 = 0.30 s | 6/12 | 17.1 | 43.46 | 39.42 | 0.0 | wall in TURN x3; wall in FOLLOW x3 |

### Servo lag (yaml steer_tau; x1 = 0.05 s)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| lag x0.5 = 0.03 s | 12/12 | 30.0 | 41.80 | 33.78 | 9.6 |  |
| lag x1 = 0.05 s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| lag x2 = 0.10 s | 12/12 | 30.0 | 41.96 | 33.78 | 9.3 |  |
| lag x4 = 0.20 s | 12/12 | 30.0 | 42.58 | 34.38 | 6.5 |  |
| lag x8 = 0.40 s | 9/12 | 22.8 | 42.50 | 37.24 | 0.0 | wall in TURN x3 |

### Servo speed limit (steer_rate; x1 = 1.2 rad/s = 69 deg/s)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| rate 4.4 rad/s | 12/12 | 30.0 | 40.90 | 32.78 | 9.5 |  |
| rate 2.4 rad/s | 12/12 | 30.0 | 41.15 | 32.88 | 10.1 |  |
| rate 1.2 rad/s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| rate 0.9 rad/s | 12/12 | 30.0 | 42.32 | 34.38 | 7.8 |  |
| rate 0.6 rad/s | 12/12 | 30.0 | 43.38 | 35.46 | 2.9 |  |
| rate 0.4 rad/s | 5/12 | 14.7 | 42.33 | 40.54 | 0.0 | wall in TURN x7 |
| rate 0.3 rad/s | 0/12 | 2.2 | - | - | 0.0 | wall in FOLLOW x6; wall in TURN x6 |
| rate 0.2 rad/s | 0/12 | 1.5 | - | - | 0.0 | wall in FOLLOW x6; wall in TURN x6 |

### Servo: slow servo (delay x3) with different fixes

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| delay x3, no change | 12/12 | 30.0 | 43.40 | 35.38 | 2.9 |  |
| delay x3, DAMPING 0 | 12/12 | 30.0 | 43.20 | 34.22 | 2.0 |  |
| delay x3, DAMPING 0.05 | 12/12 | 30.0 | 43.15 | 34.62 | 2.4 |  |
| delay x3, DAMPING 0.2 | 12/12 | 30.0 | 43.02 | 34.98 | 3.7 |  |
| delay x3, DAMPING 0.3 | 12/12 | 30.0 | 43.19 | 34.98 | 3.3 |  |
| delay x3, TURN_TOLERANCE 0.2 | 12/12 | 30.0 | 43.37 | 34.88 | 3.8 |  |
| delay x3, TURN_TOLERANCE 0.4 | 12/12 | 30.0 | 43.29 | 35.28 | 3.8 |  |
| delay x3, TURN_TOLERANCE 0.5 | 12/12 | 30.0 | 43.26 | 35.08 | 3.8 |  |
| delay x3, corner speed 0.8 | 12/12 | 30.0 | 46.40 | 37.68 | 6.1 |  |
| delay x3, corner speed 0.6 | 12/12 | 30.0 | 50.32 | 41.18 | 10.7 |  |
| delay x3, TURN_FRONT 1.25 | 12/12 | 30.0 | 43.47 | 34.84 | 2.3 |  |
| delay x3, TURN_FRONT 1.3 | 12/12 | 30.0 | 43.69 | 35.08 | 3.4 |  |
| delay x3, TURN_FRONT 1.4 | 12/12 | 30.0 | 43.89 | 35.78 | 3.8 |  |
| delay x3, cruise 0.9 | 12/12 | 30.0 | 46.13 | 37.18 | 3.9 |  |
| delay x3, cruise 0.7 | 12/12 | 30.0 | 48.38 | 38.28 | 8.1 |  |

### Maximum steering angle: what the car does (strategy still assumes 0.307 rad)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| car reaches 0.2 rad (11.5 deg) | 6/12 | 19.2 | 41.00 | 36.38 | 0.0 | wall in TURN x5; stopped short in DONE x1 |
| car reaches 0.25 rad (14.3 deg) | 11/12 | 29.8 | 43.64 | 34.88 | 1.9 | stopped short in DONE x1 |
| car reaches 0.28 rad (16.0 deg) | 12/12 | 30.0 | 42.31 | 34.18 | 6.8 |  |
| car reaches 0.307 rad (17.6 deg) | 12/12 | 30.0 | 41.85 | 33.78 | 10.1 |  |
| car reaches 0.35 rad (20.1 deg) | 12/12 | 30.0 | 41.34 | 33.08 | 8.1 |  |
| car reaches 0.4 rad (22.9 deg) | 12/12 | 30.0 | 41.34 | 33.08 | 8.1 |  |
| car reaches 0.4363 rad (25.0 deg) | 12/12 | 30.0 | 41.34 | 33.08 | 8.1 |  |

### Maximum steering angle: strategy and car agree (both change)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| both 0.2 rad (11.5 deg) | 6/12 | 19.2 | 41.00 | 36.38 | 0.0 | wall in TURN x5; stopped short in DONE x1 |
| both 0.25 rad (14.3 deg) | 11/12 | 29.8 | 43.64 | 34.88 | 1.9 | stopped short in DONE x1 |
| both 0.28 rad (16.0 deg) | 12/12 | 30.0 | 42.31 | 34.18 | 6.8 |  |
| both 0.307 rad (17.6 deg) | 12/12 | 30.0 | 41.85 | 33.78 | 10.1 |  |
| both 0.35 rad (20.1 deg) | 12/12 | 30.0 | 41.25 | 33.08 | 5.8 |  |
| both 0.4 rad (22.9 deg) | 9/12 | 27.0 | 42.58 | - | 0.5 | stopped short in DONE x3 |
| both 0.4363 rad (25.0 deg) | 6/12 | 15.5 | 43.19 | - | 0.0 | wall in FOLLOW x6 |

### Maximum steering angle: strategy assumes it but the car stays at 0.307

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| strategy assumes 0.2 rad (11.5 deg) | 8/12 | 22.5 | 41.32 | 35.58 | 0.0 | wall in TURN x3; stopped short in DONE x1 |
| strategy assumes 0.25 rad (14.3 deg) | 12/12 | 30.0 | 42.43 | 34.38 | 6.1 |  |
| strategy assumes 0.35 rad (20.1 deg) | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| strategy assumes 0.4 rad (22.9 deg) | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| strategy assumes 0.4363 rad (25.0 deg) | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |

### Commanded cruise speed (the car reaches the table value)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| cruise 0.4 m/s | 11/12 | 29.0 | 56.87 | - | 5.2 | stopped short in DONE x1 |
| cruise 0.6 m/s | 12/12 | 30.0 | 48.31 | 37.88 | 10.0 |  |
| cruise 0.8 m/s | 12/12 | 30.0 | 45.75 | 36.38 | 9.7 |  |
| cruise 1 m/s | 12/12 | 30.0 | 43.53 | 34.78 | 9.8 |  |
| cruise 1.1 m/s | 12/12 | 30.0 | 42.80 | 34.28 | 10.1 |  |
| cruise 1.28 m/s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |

### Commanded speed above the motor limit (1.28 = 100 % motor)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| cruise 1.28 | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| cruise 1.6 | 12/12 | 30.0 | 41.54 | 33.38 | 10.1 |  |
| cruise 2 | 12/12 | 30.0 | 41.34 | 33.18 | 10.1 |  |

### Speed in the corners (and the slow-down before them) and of the finish

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| corner speed 0.6 | 12/12 | 30.0 | 48.87 | 39.48 | 9.9 |  |
| corner speed 0.8 | 12/12 | 30.0 | 44.84 | 35.88 | 10.1 |  |
| corner speed 1 | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| corner speed 1.15 | 7/12 | 28.8 | 42.09 | - | 9.6 | stopped short in DONE x5 |
| corner speed 1.28 | 6/12 | 28.5 | 41.48 | - | 8.0 | stopped short in DONE x6 |
| BRAKE_MARGIN 0 m | 12/12 | 30.0 | 41.49 | 33.28 | 9.3 |  |
| BRAKE_MARGIN 0.35 m | 12/12 | 30.0 | 42.32 | 34.08 | 10.1 |  |
| BRAKE_MARGIN 0.6 m | 12/12 | 30.0 | 42.98 | 34.88 | 10.1 |  |
| BRAKE_GAIN 1 | 12/12 | 30.0 | 42.15 | 33.88 | 10.0 |  |
| BRAKE_GAIN 4 | 12/12 | 30.0 | 41.75 | 33.78 | 10.1 |  |
| finish 0.08 | 12/12 | 30.0 | 42.11 | 33.78 | 10.1 |  |
| finish 0.3 | 12/12 | 30.0 | 41.70 | 33.78 | 10.1 |  |
| finish 0.45 | 12/12 | 30.0 | 41.61 | 33.78 | 10.1 |  |
| finish 0.6 | 12/12 | 30.0 | 41.51 | 33.78 | 10.1 |  |
| finish 0.8 | 0/12 | 27.0 | - | - | 10.1 | stopped short in DONE x12 |
| FINISH_LEAD 0 m | 12/12 | 30.0 | 42.13 | 33.98 | 10.1 |  |
| FINISH_LEAD 0.15 m | 8/12 | 29.0 | 42.44 | - | 10.1 | stopped short in DONE x4 |
| FINISH_LEAD 0.3 m | 6/12 | 28.5 | 44.03 | - | 8.1 | stopped short in DONE x6 |

### Motor: the speed the car really reaches (table scale; strategy keeps its table)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| real speed x0.5 | 10/12 | 28.0 | 81.10 | 61.68 | 6.8 | stopped short in DONE x2 |
| real speed x0.7 | 12/12 | 30.0 | 57.67 | 45.58 | 9.2 |  |
| real speed x0.85 | 12/12 | 30.0 | 48.37 | 38.68 | 11.5 |  |
| real speed x1 | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| real speed x1.15 | 9/12 | 29.2 | 38.56 | - | 8.6 | stopped short in DONE x3 |
| real speed x1.3 | 8/12 | 29.0 | 33.54 | 27.38 | 3.2 | stopped short in DONE x4 |
| real speed x1.5 | 6/12 | 28.5 | 31.71 | - | 2.1 | stopped short in DONE x6 |

### Motor: slower response (drive_tau, drive_delay)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| drive_tau 0.05 s | 12/12 | 30.0 | 41.85 | 33.78 | 9.0 |  |
| drive_tau 0.15 s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| drive_tau 0.3 s | 6/12 | 28.5 | 45.00 | - | 10.4 | stopped short in DONE x6 |
| drive_tau 0.6 s | 0/12 | 27.0 | - | - | 9.0 | stopped short in DONE x12 |
| drive_delay 0 s | 12/12 | 30.0 | 41.82 | 33.78 | 10.4 |  |
| drive_delay 0.06 s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| drive_delay 0.15 s | 10/12 | 29.5 | 42.62 | - | 9.5 | stopped short in DONE x2 |
| drive_delay 0.3 s | 7/12 | 28.8 | 43.85 | - | 10.6 | stopped short in DONE x5 |
| accel_max 0.3 m/s2 | 12/12 | 30.0 | 43.07 | 34.98 | 8.1 |  |
| accel_max 0.6 m/s2 | 12/12 | 30.0 | 42.21 | 33.98 | 10.4 |  |
| accel_max 2 m/s2 | 12/12 | 30.0 | 41.58 | 33.28 | 10.0 |  |

### Lane keeping: GAP, GAIN, LIMIT

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| GAP 0.15 m | 10/12 | 29.5 | 41.72 | 33.58 | 9.1 | stopped short in DONE x2 |
| GAP 0.2 m | 12/12 | 30.0 | 41.75 | 33.78 | 10.6 |  |
| GAP 0.25 m | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| GAP 0.3 m | 12/12 | 30.0 | 41.91 | 33.78 | 9.9 |  |
| GAP 0.35 m | 12/12 | 30.0 | 41.92 | 33.98 | 9.9 |  |
| GAP 0.4 m | 12/12 | 30.0 | 41.92 | 33.98 | 11.1 |  |
| GAIN 0 | 12/12 | 30.0 | 41.85 | 33.78 | 7.7 |  |
| GAIN 0.1 | 12/12 | 30.0 | 41.87 | 33.78 | 9.5 |  |
| GAIN 0.6 | 12/12 | 30.0 | 41.82 | 33.78 | 10.3 |  |
| GAIN 1 | 12/12 | 30.0 | 41.79 | 33.78 | 10.6 |  |
| GAIN 2 | 12/12 | 30.0 | 41.66 | 33.68 | 9.5 |  |
| LIMIT 0.05 rad | 12/12 | 30.0 | 41.86 | 33.88 | 10.1 |  |
| LIMIT 0.2 rad | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| LIMIT 0.6 rad | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |

### Heading gain KP (steering per radian of heading error)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| KP 0.2 | 12/12 | 30.0 | 41.81 | 33.78 | 10.2 |  |
| KP 0.4 | 11/12 | 29.8 | 42.56 | - | 10.6 | stopped short in DONE x1 |
| KP 0.8 | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| KP 1.2 | 12/12 | 30.0 | 41.80 | 33.78 | 9.4 |  |
| KP 2 | 12/12 | 30.0 | 41.86 | 33.38 | 3.2 |  |
| KP 3 | 2/12 | 10.0 | 40.77 | 36.00 | 0.0 | wall in FOLLOW x2; wall in TURN x7 |
| KP 5 | 3/12 | 13.2 | 43.52 | - | 0.0 | wall in FOLLOW x2; wall in TURN x6 |

### Corner: DAMPING and TURN_TOLERANCE

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| DAMPING 0 | 12/12 | 30.0 | 41.79 | 33.78 | 9.6 |  |
| DAMPING 0.05 | 12/12 | 30.0 | 41.79 | 33.78 | 10.0 |  |
| DAMPING 0.1 | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| DAMPING 0.2 | 12/12 | 30.0 | 41.81 | 33.48 | 10.3 |  |
| DAMPING 0.4 | 12/12 | 30.0 | 41.70 | 33.48 | 5.7 |  |
| DAMPING 0.8 | 12/12 | 30.0 | 41.96 | 34.08 | 8.3 |  |
| TURN_TOLERANCE 0.05 rad | 12/12 | 30.0 | 42.11 | 34.28 | 4.2 |  |
| TURN_TOLERANCE 0.12 rad | 11/12 | 29.8 | 42.66 | - | 6.8 | stopped short in DONE x1 |
| TURN_TOLERANCE 0.2 rad | 10/12 | 29.5 | 42.64 | - | 10.1 | stopped short in DONE x2 |
| TURN_TOLERANCE 0.3 rad | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| TURN_TOLERANCE 0.4 rad | 12/12 | 30.0 | 41.83 | 33.78 | 10.1 |  |
| TURN_TOLERANCE 0.5 rad | 12/12 | 30.0 | 41.83 | 33.68 | 10.1 |  |
| TURN_TOLERANCE 0.8 rad | 12/12 | 30.0 | 41.75 | 33.38 | 10.1 |  |

### Corner when the car turns FASTER than the strategy believes (car wheelbase 25 % shorter): DAMPING, TURN_TOLERANCE

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| car turns 33 % faster, defaults | 12/12 | 30.0 | 41.04 | 32.88 | 6.3 |  |
| faster, DAMPING 0 | 12/12 | 30.0 | 40.96 | 32.88 | 3.5 |  |
| faster, DAMPING 0.3 | 12/12 | 30.0 | 41.09 | 32.98 | 8.2 |  |
| faster, TURN_TOLERANCE 0.12 | 6/12 | 26.2 | 44.63 | - | 2.5 | stopped short in DONE x6 |
| faster, TURN_TOLERANCE 0.5 | 12/12 | 30.0 | 41.02 | 32.58 | 9.0 |  |
| faster, TURN_TOLERANCE 0.8 | 12/12 | 30.0 | 41.05 | 32.78 | 9.0 |  |

### When to turn: TURN_FRONT, LOOKAHEAD, ARM_DISTANCE

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| TURN_FRONT 0.9 m | 12/12 | 30.0 | 43.22 | 38.38 | 9.9 |  |
| TURN_FRONT 1 m | 12/12 | 30.0 | 42.01 | 35.18 | 10.1 |  |
| TURN_FRONT 1.1 m | 12/12 | 30.0 | 41.71 | 33.78 | 9.6 |  |
| TURN_FRONT 1.19 m | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| TURN_FRONT 1.3 m | 12/12 | 30.0 | 42.14 | 33.78 | 10.1 |  |
| TURN_FRONT 1.4 m | 12/12 | 30.0 | 42.43 | 34.38 | 10.1 |  |
| TURN_FRONT 1.6 m | 12/12 | 30.0 | 42.90 | 34.78 | 10.1 |  |
| LOOKAHEAD 0.1 m | 12/12 | 30.0 | 43.30 | 35.68 | 9.4 |  |
| LOOKAHEAD 0.2 m | 12/12 | 30.0 | 43.01 | 35.18 | 10.1 |  |
| LOOKAHEAD 0.35 m | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| LOOKAHEAD 0.5 m | 9/12 | 22.8 | 41.97 | 33.78 | 0.0 | wall in FOLLOW x3 |
| LOOKAHEAD 0.8 m | 9/12 | 22.8 | 41.97 | 33.78 | 0.0 | wall in FOLLOW x3 |
| ARM_DISTANCE 0 m | 12/12 | 30.0 | 41.85 | 33.78 | 10.2 |  |
| ARM_DISTANCE 0.1 m | 12/12 | 30.0 | 41.84 | 33.78 | 10.2 |  |
| ARM_DISTANCE 0.25 m | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| ARM_DISTANCE 0.5 m | 12/12 | 30.0 | 42.07 | 34.48 | 10.1 |  |
| ARM_DISTANCE 0.8 m | 12/12 | 30.0 | 42.72 | 35.78 | 9.3 |  |
| ARM_DISTANCE 1.2 m | 0/12 | 1.0 | - | - | 0.0 | wall in TURN x12 |

### Sensors: LiDAR dropout and range noise, gyroscope bias and noise

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| LiDAR dropout 0% | 12/12 | 30.0 | 41.85 | 33.68 | 10.2 |  |
| LiDAR dropout 4% | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| LiDAR dropout 8% | 12/12 | 30.0 | 41.77 | 33.78 | 10.1 |  |
| LiDAR dropout 12% | 12/12 | 30.0 | 41.76 | 33.78 | 10.0 |  |
| LiDAR dropout 15% | 12/12 | 30.0 | 41.80 | 33.68 | 9.9 |  |
| LiDAR dropout 25% | 12/12 | 30.0 | 41.82 | 33.78 | 9.9 |  |
| LiDAR dropout 40% | 12/12 | 30.0 | 41.79 | 33.68 | 9.6 |  |
| LiDAR range noise 0 mm | 12/12 | 30.0 | 41.85 | 33.78 | 10.6 |  |
| LiDAR range noise 8 mm | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| LiDAR range noise 20 mm | 10/12 | 27.7 | 41.95 | 33.38 | 0.0 | wall in TURN x1; stopped short in DONE x1 |
| LiDAR range noise 40 mm | 0/12 | 5.3 | - | - | 0.0 | wall in TURN x7; wall in FOLLOW x3; stopped short in DONE x2 |
| LiDAR range noise 80 mm | 0/12 | 2.2 | - | - | 0.0 | wall in FOLLOW x7; wall in TURN x4; stopped short in DONE x1 |
| gyro bias 0 rad/s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| gyro bias 0.006 rad/s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| gyro bias 0.02 rad/s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| gyro bias 0.05 rad/s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| gyro bias 0.1 rad/s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| gyro noise 0 rad/s | 12/12 | 30.0 | 41.85 | 33.78 | 10.1 |  |
| gyro noise 0.0019 rad/s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| gyro noise 0.01 rad/s | 12/12 | 30.0 | 41.80 | 33.78 | 10.1 |  |
| gyro noise 0.03 rad/s | 12/12 | 30.0 | 41.82 | 33.78 | 10.0 |  |
| gyro noise 0.1 rad/s | 12/12 | 30.0 | 41.82 | 33.78 | 9.2 |  |

### Gyroscope: drift after the calibration and scale error

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| gyro drift 2e-05 rad/s per s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| gyro drift 5e-05 rad/s per s | 12/12 | 30.0 | 41.81 | 33.78 | 10.1 |  |
| gyro drift 0.0001 rad/s per s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| gyro drift 0.0002 rad/s per s | 12/12 | 30.0 | 41.76 | 33.78 | 10.1 |  |
| gyro drift 0.0005 rad/s per s | 12/12 | 30.0 | 41.86 | 33.88 | 10.1 |  |
| gyro scale 0.9 | 9/12 | 28.5 | 43.17 | - | 1.8 | stopped short in DONE x3 |
| gyro scale 0.95 | 11/12 | 29.8 | 41.88 | 33.88 | 9.8 | stopped short in DONE x1 |
| gyro scale 0.97 | 12/12 | 30.0 | 41.86 | 33.78 | 10.1 |  |
| gyro scale 0.98 | 12/12 | 30.0 | 41.87 | 33.78 | 10.1 |  |
| gyro scale 0.99 | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| gyro scale 1.01 | 12/12 | 30.0 | 41.90 | 33.78 | 10.1 |  |
| gyro scale 1.02 | 12/12 | 30.0 | 41.87 | 33.78 | 10.1 |  |
| gyro scale 1.03 | 12/12 | 30.0 | 41.86 | 33.78 | 10.1 |  |
| gyro scale 1.05 | 12/12 | 30.0 | 41.83 | 33.58 | 9.3 |  |

### Gyroscope scale error with and without the island-wall heading correction (WALL_HEADING_GAIN)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| WALL_HEADING_GAIN 0 | 12/12 | 30.0 | 41.86 | 33.78 | 10.5 |  |
| WALL_HEADING_GAIN 0.1 | 12/12 | 30.0 | 41.90 | 33.78 | 10.8 |  |
| WALL_HEADING_GAIN 0.3 | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| WALL_HEADING_GAIN 0.6 | 12/12 | 30.0 | 41.85 | 33.78 | 9.4 |  |
| WALL_HEADING_GAIN 1 | 12/12 | 30.0 | 41.81 | 33.68 | 8.3 |  |
| gyro scale 0.9, gain 0 (off) | 0/12 | 3.3 | - | - | 0.0 | wall in FOLLOW x12 |
| gyro scale 0.9, gain 0.3 | 9/12 | 28.5 | 43.17 | - | 1.8 | stopped short in DONE x3 |
| gyro scale 0.94, gain 0 (off) | 0/12 | 6.2 | - | - | 0.0 | wall in FOLLOW x10; wall in TURN x2 |
| gyro scale 0.94, gain 0.3 | 10/12 | 29.5 | 43.46 | - | 9.2 | stopped short in DONE x2 |
| gyro scale 0.97, gain 0 (off) | 0/12 | 13.0 | - | - | 0.0 | wall in FOLLOW x6; wall in TURN x6 |
| gyro scale 0.97, gain 0.3 | 12/12 | 30.0 | 41.86 | 33.78 | 10.1 |  |
| gyro scale 1.03, gain 0 (off) | 2/12 | 20.8 | 35.38 | 35.48 | 0.0 | wall in TURN x4; wall in FOLLOW x6 |
| gyro scale 1.03, gain 0.3 | 12/12 | 30.0 | 41.86 | 33.78 | 10.1 |  |
| gyro scale 1.06, gain 0 (off) | 0/12 | 11.7 | - | - | 0.0 | wall in FOLLOW x10; wall in TURN x2 |
| gyro scale 1.06, gain 0.3 | 12/12 | 30.0 | 41.78 | 33.68 | 9.7 |  |
| gyro scale 1.1, gain 0 (off) | 0/12 | 8.0 | - | - | 0.0 | wall in FOLLOW x9; wall in TURN x3 |
| gyro scale 1.1, gain 0.3 | 12/12 | 30.0 | 41.91 | 33.48 | 9.3 |  |
| gyro drift 0.0002 rad/s per s, gain 0 (off) | 8/12 | 28.6 | 40.39 | 33.88 | 0.0 | wall in FOLLOW x1; stopped short in DONE x3 |
| gyro drift 0.0005 rad/s per s, gain 0 (off) | 4/12 | 24.3 | 38.81 | 34.58 | 0.0 | wall in FOLLOW x5; wall in FINISH x1; stopped short in DONE x2 |

### Calibration time (CALIBRATE_SECONDS) with different gyro biases

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| calibrate 0.05 s | 12/12 | 30.0 | 41.85 | 33.78 | 9.9 |  |
| calibrate 0.1 s | 12/12 | 30.0 | 41.86 | 33.78 | 10.1 |  |
| calibrate 0.3 s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| calibrate 1 s | 12/12 | 30.0 | 42.52 | 34.38 | 9.7 |  |
| calibrate 2 s | 12/12 | 30.0 | 43.51 | 35.38 | 10.8 |  |
| calibrate 0.05 s, gyro noise 0.01 | 12/12 | 30.0 | 41.81 | 33.78 | 10.2 |  |
| calibrate 0.3 s, gyro noise 0.01 | 12/12 | 30.0 | 41.80 | 33.78 | 10.1 |  |

### Camera: slower to see the direction

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| camera ready after 0.3 s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| camera ready after 1 s | 12/12 | 30.0 | 42.51 | 34.38 | 9.6 |  |
| camera ready after 3 s | 12/12 | 30.0 | 44.59 | 36.28 | 10.0 |  |
| camera ready after 8 s | 12/12 | 30.0 | 49.50 | 41.38 | 10.2 |  |
| camera ready after 14 s | 12/12 | 30.0 | 55.48 | 47.28 | 10.3 |  |
| camera ready after 16 s | 0/12 | 0.0 | - | - | 17.6 | stopped short in DIRECTION_FAULT x12 |
| camera 6 fps | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| camera 3 fps | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| camera 1 fps | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |

### Control latency (Pi/ROS between the strategy and the actuators)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| latency 0 ms | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| latency 20 ms | 12/12 | 30.0 | 42.06 | 33.88 | 9.2 |  |
| latency 50 ms | 12/12 | 30.0 | 42.48 | 34.48 | 9.5 |  |
| latency 100 ms | 12/12 | 30.0 | 43.12 | 35.28 | 4.9 |  |
| latency 200 ms | 5/12 | 21.5 | 44.00 | - | 0.0 | wall in TURN x4; stopped short in DONE x1 |

### Wheelbase: a longer or shorter car, code NOT updated vs updated

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| wheelbase 9.7 cm, strategy not updated | 12/12 | 30.0 | 41.35 | 33.08 | 7.7 |  |
| wheelbase 9.7 cm, WHEELBASE updated | 12/12 | 30.0 | 41.05 | 32.88 | 6.1 |  |
| wheelbase 11.7 cm, strategy not updated | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| wheelbase 14.7 cm, strategy not updated | 10/12 | 29.8 | 43.68 | 34.98 | 0.5 | stopped short in DONE x1 |
| wheelbase 14.7 cm, WHEELBASE updated | 9/12 | 29.5 | 43.71 | 34.98 | 0.7 | stopped short in DONE x2 |
| wheelbase 17.7 cm, strategy not updated | 3/12 | 18.4 | 40.45 | - | 0.0 | wall in TURN x6; stopped short in DONE x2 |
| wheelbase 17.7 cm, WHEELBASE updated | 3/12 | 19.5 | 42.86 | - | 0.0 | wall in TURN x5; stopped short in DONE x3 |
| wheelbase 23.7 cm, strategy not updated | 0/12 | 6.3 | - | - | 0.0 | wall in TURN x10; stopped short in DONE x2 |
| wheelbase 23.7 cm, WHEELBASE updated | 1/12 | 6.6 | 39.42 | 39.42 | 0.0 | wall in TURN x10; stopped short in DONE x1 |

### Faster candidates, 3 seeds (use --seeds 0,1,2)

Seeds 0,1,2, 36 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| baseline | 36/36 | 30.0 | 41.85 | 33.78 | 9.9 |  |
| corner 1.28 + finish 0.6 (fast) | 18/36 | 28.5 | 41.47 | - | 8.0 | stopped short in DONE x18 |
| corner 1.15 + finish 0.45 (safer) | 20/36 | 28.7 | 42.23 | - | 8.8 | stopped short in DONE x16 |
| finish 0.45 only | 36/36 | 30.0 | 41.64 | 33.78 | 9.9 |  |
| fast + GAIN 0.6 | 18/36 | 28.5 | 41.39 | - | 8.1 | stopped short in DONE x18 |
| fast + steering 25 deg (car and strategy) | 9/36 | 14.8 | 42.68 | - | 0.0 | wall in FOLLOW x18; stopped short in DONE x9 |

### Baseline vs the faster candidates when something else is off

Seeds 0,1,2, 36 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| baseline, motor x0.85 | 36/36 | 30.0 | 48.38 | 38.68 | 11.5 |  |
| fast, motor x0.85 | 18/36 | 28.5 | 47.77 | - | 11.6 | stopped short in DONE x18 |
| safer, motor x0.85 | 27/36 | 29.2 | 47.23 | - | 11.0 | stopped short in DONE x9 |
| baseline, motor x1.15 | 29/36 | 29.4 | 37.74 | 29.98 | 7.9 | stopped short in DONE x7 |
| fast, motor x1.15 | 18/36 | 28.5 | 36.77 | - | 4.0 | stopped short in DONE x18 |
| safer, motor x1.15 | 18/36 | 28.5 | 38.00 | - | 5.3 | stopped short in DONE x18 |
| baseline, servo delay x2 | 36/36 | 30.0 | 42.54 | 34.38 | 6.9 |  |
| fast, servo delay x2 | 18/36 | 28.5 | 42.12 | - | 4.4 | stopped short in DONE x18 |
| safer, servo delay x2 | 18/36 | 28.5 | 43.56 | - | 6.6 | stopped short in DONE x18 |
| baseline, servo rate 0.6 | 36/36 | 30.0 | 43.39 | 35.46 | 2.9 |  |
| fast, servo rate 0.6 | 17/36 | 27.2 | 42.58 | - | 0.0 | wall in TURN x2; stopped short in DONE x17 |
| safer, servo rate 0.6 | 32/36 | 29.7 | 41.86 | 34.14 | 1.5 | stopped short in DONE x4 |
| baseline, LiDAR dropout 12% | 36/36 | 30.0 | 41.83 | 33.78 | 10.0 |  |
| fast, LiDAR dropout 12% | 18/36 | 28.5 | 41.44 | - | 6.7 | stopped short in DONE x18 |

### Gazebo rounds (`raw/gazebo.tsv`: name, direction, points, time from Start s, laps s, judge, extra)

```
base	ccw	30 de 30	35.56	11.4 / 11.0 / 11.0	sí   margen mínimo 17.7 cm	
base	cw	30 de 30	35.53	11.4 / 11.0 / 11.0	sí   margen mínimo 18.0 cm
```
