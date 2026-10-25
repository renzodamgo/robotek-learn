# Tuning results (chapter 12, "Afinar")

Raw material for `Portfolio-Astro/src/content/learn/steps/open-strategy/paso-12-afinar.md`. Everything here was measured
with the reference strategy `src/my_open` (the course solution) in the **Open lab** (`robotek_evaluation.open_lab`, no
Gazebo) unless it says Gazebo. Nothing was run on the real car. The raw tables are in `raw/*.md`, the per-round data in
`raw/*.json`; they are appended at the end of this file unchanged.

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
* Baseline: 12/12 valid, mean time 40.75 s, 32.58 s on square100, smallest clearance 9.5 cm (seed 0). With seeds
  0,1,2: 36/36, 40.77 s, 32.58 s, 8.2 cm. Gazebo (square100, no window, `gazebo.tsv`): ccw 30 of 30 points, 34.42 s,
  laps 11.0 / 10.5 / 10.7 s, margin 9.5 cm; cw 30 of 30, 34.72 s, laps 10.9 / 10.6 / 10.9 s, margin 14.2 cm.
  The lab is about 1.8 s faster than Gazebo on square100 ccw (32.58 vs 34.42).

## Findings, by question

### 1. A slower servo (delay, rate)

Base delay 0.06 s (lab 0.10 s). Rows are the yaml value x1.5 / x2 / x3 / x5 = 0.09 / 0.12 / 0.18 / 0.30 s.

| Servo | valid | mean time | smallest clearance | source |
|---|---:|---:|---:|---|
| x1 (0.06 s) | 12/12 | 40.75 s | 9.5 cm | `raw/*.md` baseline |
| x1.5 (0.09 s) | 12/12 | 41.12 s (+0.4) | 7.7 cm | summary; **re-checked** (lab, same seed, see below) |
| x2 (0.12 s) | 12/12 | 41.54 s (+0.8) | 5.9 cm | `raw/candidates_robust.md` |
| x3 (0.18 s) | 12/12 | 42.24 s (+1.5) | 2.0 cm | `raw/servo_fix.md` |
| x5 (0.30 s) | 2/12 | - | - | summary says 10 of 12 fail in corners; **re-checked**: 2/12 valid |

Gazebo, x2 (`actuator_params:=steer_delay=0.12`), ccw: 30 of 30, 35.92 s (base 34.42 s, +1.5 s), laps 11.2 / 11.1 /
11.3 s, margin 15.8 cm (`raw/gazebo.tsv`). In Gazebo the margin did not shrink as in the lab (9.5 cm base, 15.8 cm
slow); the lab margin is pessimistic or Gazebo has different margins: one round, not enough to say.

Servo rate limit (`steer_rate`, base 1.2 rad/s = 69 deg/s): 1.2 ok; 0.6 rad/s: 12/12, 42.24 s, clearance 1.7 cm
(`raw/candidates_robust.md`); 0.4: summary says 5 of 12 fail, **re-checked**: 7/12 valid; 0.3: all fail (summary, no raw
file, not re-checked).

**A higher DAMPING does not repair a slow servo** (`raw/servo_fix.md`, delay x3, base row 12/12, 42.24 s, 2.0 cm):
DAMPING 0: 10/12; 0.05: 11/12; 0.2: 9/12 (clearance 0.6 cm); 0.3: 9/12. `EXIT_ANGLE` 0.1/0.4/0.6 and `TURN_FRONT`
1.25/1.3/1.4 change nothing (42.24 to 42.34 s, 2.0 cm). What gives margin is **less speed**: cruise 0.9 -> 6.3 cm
(48.24 s), cruise 0.7 -> 7.9 cm (52.42 s).

The raw folder has no `servo_delay`, `servo_lag` or `servo_rate` tables: the experimenter's `lab_sweep.py` defines the
groups (`servo_delay`, `servo_lag`, `servo_rate`), but their `.md/.json` are not in `raw/`. Their numbers in this file
come from rows of other tables (x2, x3, rate 0.6), from the experimenter's summary, or from the 3 re-checks.

### 2. Maximum steering angle

`raw/steer_limit.md` (the car reaches less/more, the strategy still assumes 0.307 rad = 17.6 degrees): 0.25 rad
(14.3 deg) 12/12, 41.60 s, 1.6 cm; 0.28 rad 12/12, 5.5 cm; 0.20 rad (11.5 deg) 7/12 valid (wall in TURN x4); 0.35 / 0.40 /
0.4363 rad 12/12, 40.41 s, 7.1 cm (a car that can turn more barely changes anything: -0.35 s).
`raw/steer_assumed.md` (strategy and car agree): 0.25 rad 12/12, 41.60 s; 0.20 rad 7/12; 0.35 rad 40.35 s; 0.4363 rad
(25 deg) 39.93 s, 6.8 cm: only -0.8 s with respect to 40.76 s. `raw/steer_assumed_only.md` (the strategy assumes
more than the car has): assuming 0.25 rad 12/12 (4.2 cm), 0.20 rad 7/12; assuming 0.35 to 0.4363 rad is identical to the
base (the strategy clamps at what is useful).

### 3. Speed

Real speed = `speed_table_real` times a factor (`--act-scale speed_table_real=K`), the strategy keeps its table
(`raw/motor.md`): x0.5 76.12 s (5.6 cm); x0.7 55.98 s; x0.85 47.03 s (12.1 cm); x1 40.75 s; x1.15 35.99 s (7.0 cm);
x1.3 32.41 s (1.5 cm, still valid); x1.5 2/12 (wall in TURN x8). Updating the strategy table too
(`raw/motor_aware.md`): x0.7 55.89 s; x0.85 47.08 s; x1.15 36.19 s (8.7 cm); x1.3 32.76 s (3.9 cm, valid): same time,
a little more margin at high speed.

Commanded cruise (`raw/cruise.md`, `raw/cruise_high.md`): 0.4 70.56 s; 0.6 53.33 s; 0.8 48.60 s; 1.0 44.92 s; 1.1
43.30 s; 1.28 40.75 s. Above the motor limit (1.28 = 100 % motor, `drive_command_limit`), 3 seeds: 1.6 40.63 s,
2.0 40.33 s versus 40.77 s at 1.28: it does not help (-0.4 s at best).

Motor response (`raw/motor_slow.md`): `drive_tau` 0.05/0.15/0.3 s no change; 0.6 s: 0/12 (no finish in DONE x12);
`drive_delay` 0 to 0.3 s no change; `accel_max` 0.3 m/s2 41.80 s, 0.6 41.06 s, 2 m/s2 40.53 s.

### 4. What breaks it

* `KP` (`raw/kp.md`): 0.2 to 1.2 12/12 (0.2: 2.2 cm); 2: 6/12; 3: 3/12; 5: 2/12 (wall in TURN).
* LiDAR range noise (`raw/sensors.md`): 0 to 8 mm 12/12; 20 mm 10/12; 40 mm 1/12; 80 mm 0/12.
* LiDAR dropout: 0 to 15 % 12/12 (15 %: 7.8 cm); 25 % 2/12; 40 % 2/12.
* Gyro bias 0 to 0.1 rad/s: no effect (calibration removes it). Gyro noise up to 0.03 rad/s 12/12; 0.1 rad/s 11/12.
* Gyro scale (`raw/gyro_drift.md`): 0.99: 12/12 (2.6 cm), 1.01: 12/12 (5.7 cm); 0.98: 1/12; 1.02: 5/12; 0.97: 2/12;
  0.95 and 0.9: 0/12; 1.03 and 1.05: 0/12.
* Gyro drift after the calibration (rad/s per s): 0.00002 to 0.0001 12/12; 0.0002: 11/12 (1.6 cm); 0.0005: 4/12.
* Calibration (`raw/calibration.md`): 0.05 to 0.3 s same (40.73 to 40.75 s); 1 s 41.52; 2 s 42.48. With gyro noise 0.01 rad/s,
  0.05 s calibration: 2/12 valid; 0.3 s: 12/12.
* Camera (`raw/camera.md`): the direction ready after 0.3 / 1 / 3 / 8 / 14 s gives 40.75 / 41.52 / 43.45 / 48.39 / 54.41 s, all
  12/12; 16 s: 0/12 (`DIRECTION_FAULT` x12, the 15 s timeout). Frame rate 6 / 3 / 1 fps: no change.
* Control latency (`raw/latency.md`): 0 ms 40.75 s; 20 ms 6.9 cm; 50 ms 7.6 cm; 100 ms 12/12, 41.89 s, 3.0 cm; 200 ms 5/12.
* Wheelbase (`raw/wheelbase.md`): 9.7 cm 12/12 (6.0 cm not updated, 7.1 cm updated); 11.7 cm (base); 14.7 cm 9/12; 17.7 cm
  7/12; 23.7 cm 2/12. Updating `WHEELBASE` in the code makes no difference: a longer car simply does not fit the corner
  with the same steering limit.

### 5. Constants that hardly change anything

`raw/lane.md`: `GAP` 0.15 to 0.40 m 12/12 (40.61 to 40.87 s); `GAIN` 0 to 2 12/12 (40.61 to 40.78 s; 2: 7.4 cm); `LIMIT`
0.05 to 0.6 identical. `raw/damping.md`: `DAMPING` 0 to 0.4 12/12 (40.62 to 40.84 s; 0.8: 4.9 cm); `EXIT_ANGLE` 0.02 to 0.9
identical to the digit. `raw/fade.md`: with a car that turns 33 % faster than assumed, DAMPING 0/0.3 and EXIT_ANGLE
change nothing (clearance 4.3 cm either way; EXIT_ANGLE 0.9 gives 6.1 cm). `raw/gate.md`: `TURN_FRONT` 1.1 to 1.6 m
same; 1.0 m 41.47 s; 0.9 m 42.86 s; `LOOKAHEAD` 0.1/0.2 m +1.8/+1.9 s, 0.5 m 4.2 cm, 0.8 m 9/12; `ARM_DISTANCE` 0 to 0.25 m same,
0.5 m 40.89 s, **0.8 m 4/12, 1.2 m 0/12**.

### 6. Faster and still safe (`search_speed.py`, `raw/search_state.json`, `raw/candidates*.md`)

A coordinate search (two seeds, 24 rounds per evaluation, must be 24/24 valid with clearance >= 5 cm) found:
`LAST_CORNER_SPEED` 0.6 -> 1.1 and `FINISH_SPEED` 0.15 -> 0.6 (and `GAIN` 0.6, which the 3-seed check rejected: 35/36).
The search stopped after one pass (`done: false`, cursor 7 of 11 constants): it was not run to convergence.
Search path: last corner 0.8 / 0.9 / 1.0 / 1.1 = 40.02 / 39.72 / 39.41 / 39.16 s; then finish 0.3 / 0.45 / 0.6 =
39.07 / 39.01 / 38.93 s; finish 0.8: 0 of 24 valid; `FINISH_LEAD` 0.15: 14 of 24; `BRAKE_GAIN` and `BRAKE_MARGIN` do not change
anything; `CALIBRATE_SECONDS` 0.1 or 0.2 and `GAP` 0.15 / 0.2 lose a few rounds with two seeds.

Three seeds, 36 rounds (`raw/candidates.md`): baseline 40.77 s; **1.1 + 0.6: 36/36, 38.93 s (-1.84 s, -4.5 %), 8.2 cm**;
**0.9 + 0.45 (safer): 36/36, 39.34 s (-1.43 s), 8.2 cm**; 1.1 + 0.6 + GAIN 0.6: 35/36 (stopped short in DONE x1);
1.1 + 0.6 + steering 25 degrees (car and strategy): 36/36, 38.03 s, 6.7 cm; last corner 1.28 + finish 0.6: 18/36.

When something else is off (`raw/candidates_robust.md`, seed 0, 12 rounds):

| Case | baseline | 1.1 + 0.6 | 0.9 + 0.45 |
|---|---:|---:|---:|
| motor x0.85 | 12/12, 47.03 s | 12/12, 44.78 s | 12/12, 45.28 s |
| motor x1.15 | 12/12, 35.99 s | **7/12**, stopped short in DONE x5 | 12/12, 34.83 s |
| servo delay x2 | 12/12, 41.54 s, 5.9 cm | 12/12, 39.62 s | 12/12, 40.05 s |
| servo rate 0.6 | 12/12, 42.24 s, 1.7 cm | 12/12, 40.31 s | 12/12, 40.74 s |
| LiDAR dropout 12 % | 12/12, 40.67 s | 12/12, 38.85 s | - |

Single-constant sweeps (seed 0): last corner 0.3 / 0.6 / 0.9 / 1.28 = 42.21 / 40.75 / 39.72 s / 6 of 12 valid (no
finish in DONE x6); finish 0.08 / 0.3 / 0.6 = 42.20 / 40.12 / 39.54 s.

## Where numbers come from

* Everything in the tables above is in `raw/*.md` or `raw/*.json`, except what is marked "summary" (from the
  experimenter's summary and not in the raw folder): the servo-rate row 0.3 rad/s ("all fail"), and the statements
  that the Gazebo comparisons were limited to the three rows of `raw/gazebo.tsv`.
* Re-checked by the writer in the Open lab (seed 0, 12 rounds, `--jobs 12`, nothing else changed): servo delay x1.5
  (`--act steer_delay=0.13 --act steer_tau=0.06 --act steer_rate=1.2`): 12/12 valid, mean 41.12 s, smallest clearance
  7.66 cm; servo delay x5 (`steer_delay=0.34`): 2/12 valid; servo rate 0.4 rad/s: 7/12 valid, mean 40.97 s over the
  valid-30-point rounds. All three agree with the summary.
* Differences between the summary and the raw data, kept for the record:
  * "`GAP`, `GAIN`, `LIMIT`, `ARM_DISTANCE`, `EXIT_ANGLE` hardly change anything" is true inside the tested range, but
    `ARM_DISTANCE` 0.8 m and 1.2 m fail (4/12 and 0/12), and `LOOKAHEAD` 0.8 m fails 3 of 12.
  * "Gyro drift >= 0.0002 fails": at 0.0002 only 1 of 12 rounds fails (stopped short in DONE); 0.0005 fails 8 of 12.
  * "Motor x1.3 (margin 1.5 cm)" is valid but at the edge of the 1 cm criterion.
  * The summary does not mention `drive_tau` 0.6 s (0/12), the wheelbase sweep, the camera frame rate, the calibration
    sweep or the gyro noise: they are in the raw tables and in this file.

## Reproduce

```
source tools/macos_env.sh ~/Projects/WRO/.mamba/jazzy; source install/setup.zsh
python3 course/maintainers/tuning/lab_sweep.py --list
python3 course/maintainers/tuning/lab_sweep.py servo_fix --seeds 0 --jobs 12
python3 course/maintainers/tuning/search_speed.py --seeds 0,1 --jobs 12
bash course/maintainers/tuning/gazebo_round.sh delay-x2 ccw actuator_params:=steer_delay=0.12
```

New options used (all off by default): `open_lab.py` `--set MODULE.NAME=VALUE`, `--act NAME=VALUE`,
`--act-scale NAME=FACTOR`, `--latency S`, `--seeds 0,1,2`, `--jobs N`, lab constants `GYRO_DRIFT` and `GYRO_SCALE`;
`sim.launch.py` `actuator_params:=name=value,name=value` (also `speed_scale=K`, lists with slashes) and
`actuator_params_file:=file.yaml`; `round.launch.py` passes them and adds `set:='module.NAME=value;module.NAME=value'`.

---

# Appendix: the raw tables, unchanged (`raw/*.md`)

### Calibration time (CALIBRATE_SECONDS) with different gyro biases

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| calibrate 0.05 s | 12/12 | 30.0 | 40.73 | 32.88 | 8.3 |  |
| calibrate 0.1 s | 12/12 | 30.0 | 40.74 | 32.68 | 9.4 |  |
| calibrate 0.3 s | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| calibrate 1 s | 12/12 | 30.0 | 41.52 | 33.18 | 7.4 |  |
| calibrate 2 s | 12/12 | 30.0 | 42.48 | 34.28 | 7.9 |  |
| calibrate 0.05 s, gyro noise 0.01 | 2/12 | 19.8 | 35.38 | - | 0.0 | wall in FOLLOW x8; wall in TURN x1; stopped short in DONE x1 |
| calibrate 0.3 s, gyro noise 0.01 | 12/12 | 30.0 | 40.76 | 32.68 | 9.0 |  |

### Camera: slower to see the direction

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| camera ready after 0.3 s | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| camera ready after 1 s | 12/12 | 30.0 | 41.52 | 33.28 | 7.4 |  |
| camera ready after 3 s | 12/12 | 30.0 | 43.45 | 35.38 | 9.5 |  |
| camera ready after 8 s | 12/12 | 30.0 | 48.39 | 40.28 | 9.5 |  |
| camera ready after 14 s | 12/12 | 30.0 | 54.41 | 46.28 | 8.0 |  |
| camera ready after 16 s | 0/12 | 0.0 | - | - | 17.6 | stopped short in DIRECTION_FAULT x12 |
| camera 6 fps | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| camera 3 fps | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| camera 1 fps | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |

### Faster candidates, 3 seeds (use --seeds 0,1,2)

Seeds 0,1,2, 36 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| baseline | 36/36 | 30.0 | 40.77 | 32.58 | 8.2 |  |
| last corner 1.1 + finish 0.6 (search result) | 36/36 | 30.0 | 38.93 | 31.58 | 8.2 |  |
| last corner 0.9 + finish 0.45 (safer) | 36/36 | 30.0 | 39.34 | 31.78 | 8.2 |  |
| search result + GAIN 0.6 | 35/36 | 29.9 | 38.87 | 31.48 | 9.1 | stopped short in DONE x1 |
| search result + steering 25 deg (car and strategy) | 36/36 | 30.0 | 38.03 | 30.88 | 6.7 |  |
| last corner 1.28 (as cruise) + finish 0.6 | 18/36 | 28.5 | 41.49 | - | 8.2 | stopped short in DONE x18 |

### Baseline vs search result when something else is off

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| baseline, motor x0.85 | 12/12 | 30.0 | 47.03 | 37.18 | 12.1 |  |
| search result, motor x0.85 | 12/12 | 30.0 | 44.78 | 35.88 | 12.2 |  |
| safer, motor x0.85 | 12/12 | 30.0 | 45.28 | 36.28 | 12.2 |  |
| baseline, motor x1.15 | 12/12 | 30.0 | 35.99 | 28.58 | 7.0 |  |
| search result, motor x1.15 | 7/12 | 28.8 | 36.74 | - | 7.0 | stopped short in DONE x5 |
| safer, motor x1.15 | 12/12 | 30.0 | 34.83 | 27.98 | 7.0 |  |
| baseline, servo delay x2 | 12/12 | 30.0 | 41.54 | 33.28 | 5.9 |  |
| search result, servo delay x2 | 12/12 | 30.0 | 39.62 | 32.18 | 5.9 |  |
| safer, servo delay x2 | 12/12 | 30.0 | 40.05 | 32.38 | 5.9 |  |
| baseline, servo rate 0.6 | 12/12 | 30.0 | 42.24 | 34.18 | 1.7 |  |
| search result, servo rate 0.6 | 12/12 | 30.0 | 40.31 | 32.98 | 1.7 |  |
| safer, servo rate 0.6 | 12/12 | 30.0 | 40.74 | 33.18 | 1.7 |  |
| baseline, LiDAR dropout 12% | 12/12 | 30.0 | 40.67 | 32.58 | 9.1 |  |
| search result, LiDAR dropout 12% | 12/12 | 30.0 | 38.85 | 31.58 | 9.1 |  |

### Commanded cruise speed (the car reaches the table value)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| cruise 0.4 m/s | 12/12 | 30.0 | 70.56 | 55.88 | 4.7 |  |
| cruise 0.6 m/s | 12/12 | 30.0 | 53.33 | 42.38 | 9.2 |  |
| cruise 0.8 m/s | 12/12 | 30.0 | 48.60 | 38.48 | 10.1 |  |
| cruise 1 m/s | 12/12 | 30.0 | 44.92 | 35.78 | 10.8 |  |
| cruise 1.1 m/s | 12/12 | 30.0 | 43.30 | 34.68 | 10.4 |  |
| cruise 1.28 m/s | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |

### Commanded speed above the motor limit (1.28 = 100 % motor)

Seeds 0,1,2, 36 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| cruise 1.28 | 36/36 | 30.0 | 40.77 | 32.58 | 8.2 |  |
| cruise 1.6 | 36/36 | 30.0 | 40.63 | 32.58 | 8.2 |  |
| cruise 2 | 36/36 | 30.0 | 40.33 | 32.28 | 8.2 |  |

### Corner: DAMPING and EXIT_ANGLE

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| DAMPING 0 | 12/12 | 30.0 | 40.62 | 32.48 | 8.1 |  |
| DAMPING 0.05 | 12/12 | 30.0 | 40.73 | 32.48 | 9.0 |  |
| DAMPING 0.1 | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| DAMPING 0.2 | 12/12 | 30.0 | 40.82 | 32.68 | 8.6 |  |
| DAMPING 0.4 | 12/12 | 30.0 | 40.84 | 32.58 | 7.9 |  |
| DAMPING 0.8 | 12/12 | 30.0 | 40.63 | 31.68 | 4.9 |  |
| EXIT_ANGLE 0.02 rad | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| EXIT_ANGLE 0.1 rad | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| EXIT_ANGLE 0.25 rad | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| EXIT_ANGLE 0.4 rad | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| EXIT_ANGLE 0.6 rad | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| EXIT_ANGLE 0.9 rad | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |

### Corner when the car turns FASTER than the strategy believes (car wheelbase 25 % shorter): DAMPING, EXIT_ANGLE

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| car turns 33 % faster, defaults | 12/12 | 30.0 | 39.97 | 31.88 | 4.3 |  |
| faster, DAMPING 0 | 12/12 | 30.0 | 40.01 | 31.88 | 4.3 |  |
| faster, DAMPING 0.3 | 12/12 | 30.0 | 39.97 | 31.88 | 4.3 |  |
| faster, EXIT_ANGLE 0.02 | 12/12 | 30.0 | 39.97 | 31.88 | 4.3 |  |
| faster, EXIT_ANGLE 0.6 | 12/12 | 30.0 | 40.00 | 31.88 | 4.8 |  |
| faster, EXIT_ANGLE 0.9 | 12/12 | 30.0 | 40.01 | 31.88 | 6.1 |  |
| faster, DAMPING 0 + EXIT_ANGLE 0.02 | 12/12 | 30.0 | 40.01 | 31.88 | 4.3 |  |

### When to turn: TURN_FRONT, LOOKAHEAD, ARM_DISTANCE

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| TURN_FRONT 0.9 m | 12/12 | 30.0 | 42.86 | 38.68 | 7.7 |  |
| TURN_FRONT 1 m | 12/12 | 30.0 | 41.47 | 34.78 | 9.5 |  |
| TURN_FRONT 1.1 m | 12/12 | 30.0 | 40.71 | 32.58 | 9.5 |  |
| TURN_FRONT 1.19 m | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| TURN_FRONT 1.3 m | 12/12 | 30.0 | 40.80 | 32.68 | 9.5 |  |
| TURN_FRONT 1.4 m | 12/12 | 30.0 | 40.83 | 32.68 | 9.5 |  |
| TURN_FRONT 1.6 m | 12/12 | 30.0 | 40.87 | 32.68 | 9.5 |  |
| LOOKAHEAD 0.1 m | 12/12 | 30.0 | 42.69 | 34.98 | 7.6 |  |
| LOOKAHEAD 0.2 m | 12/12 | 30.0 | 42.58 | 34.78 | 7.7 |  |
| LOOKAHEAD 0.35 m | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| LOOKAHEAD 0.5 m | 12/12 | 30.0 | 40.60 | 32.38 | 4.2 |  |
| LOOKAHEAD 0.8 m | 9/12 | 22.8 | 40.81 | 32.38 | 0.0 | wall in FOLLOW x2; wall in TURN x1 |
| ARM_DISTANCE 0 m | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| ARM_DISTANCE 0.1 m | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| ARM_DISTANCE 0.25 m | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| ARM_DISTANCE 0.5 m | 12/12 | 30.0 | 40.89 | 33.08 | 9.5 |  |
| ARM_DISTANCE 0.8 m | 4/12 | 16.2 | 46.03 | - | 0.0 | wall in FOLLOW x8 |
| ARM_DISTANCE 1.2 m | 0/12 | 1.0 | - | - | 0.0 | wall in TURN x12 |

### Gyroscope: drift after the calibration and scale error

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| gyro drift 2e-05 rad/s per s | 12/12 | 30.0 | 40.72 | 32.58 | 9.5 |  |
| gyro drift 5e-05 rad/s per s | 12/12 | 30.0 | 40.72 | 32.68 | 8.9 |  |
| gyro drift 0.0001 rad/s per s | 12/12 | 30.0 | 40.77 | 32.78 | 8.8 |  |
| gyro drift 0.0002 rad/s per s | 11/12 | 29.6 | 39.63 | 33.08 | 1.6 | stopped short in DONE x1 |
| gyro drift 0.0005 rad/s per s | 4/12 | 24.8 | 35.53 | 34.18 | 0.0 | wall in FOLLOW x4; wall in TURN x1; stopped short in DONE x3 |
| gyro scale 0.9 | 0/12 | 3.5 | - | - | 0.0 | wall in FOLLOW x12 |
| gyro scale 0.95 | 0/12 | 11.0 | - | - | 0.0 | wall in FINISH x1; wall in FOLLOW x9; wall in TURN x1; stopped short in DONE x1 |
| gyro scale 0.97 | 2/12 | 15.1 | 35.48 | 35.68 | 0.0 | wall in FOLLOW x8; wall in TURN x2 |
| gyro scale 0.98 | 1/12 | 20.0 | 33.18 | 33.18 | 0.0 | wall in FOLLOW x10; wall in TURN x1 |
| gyro scale 0.99 | 12/12 | 30.0 | 39.65 | 32.28 | 2.6 |  |
| gyro scale 1.01 | 12/12 | 30.0 | 41.50 | 33.78 | 5.7 |  |
| gyro scale 1.02 | 5/12 | 25.4 | 40.32 | 37.58 | 0.0 | wall in FOLLOW x3; wall in TURN x1; stopped short in DONE x3 |
| gyro scale 1.03 | 0/12 | 19.4 | - | - | 0.0 | wall in FOLLOW x10; stopped short in DONE x2 |
| gyro scale 1.05 | 0/12 | 11.8 | - | - | 0.0 | wall in FOLLOW x11; stopped short in DONE x1 |

### Heading gain KP (steering per radian of heading error)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| KP 0.2 | 12/12 | 30.0 | 40.90 | 32.68 | 2.2 |  |
| KP 0.4 | 12/12 | 30.0 | 40.66 | 31.68 | 7.2 |  |
| KP 0.8 | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| KP 1.2 | 12/12 | 30.0 | 40.66 | 32.48 | 9.5 |  |
| KP 2 | 6/12 | 16.8 | 38.18 | 33.78 | 0.0 | wall in TURN x6 |
| KP 3 | 3/12 | 16.4 | 40.30 | 38.18 | 0.0 | wall in TURN x7; wall in FOLLOW x1 |
| KP 5 | 2/12 | 14.6 | 44.78 | 39.48 | 0.0 | wall in FOLLOW x4; wall in TURN x6 |

### Lane keeping: GAP, GAIN, LIMIT

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| GAP 0.15 m | 12/12 | 30.0 | 40.61 | 32.38 | 8.4 |  |
| GAP 0.2 m | 12/12 | 30.0 | 40.74 | 32.58 | 8.8 |  |
| GAP 0.25 m | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| GAP 0.3 m | 12/12 | 30.0 | 40.80 | 32.58 | 9.0 |  |
| GAP 0.35 m | 12/12 | 30.0 | 40.86 | 32.88 | 8.6 |  |
| GAP 0.4 m | 12/12 | 30.0 | 40.87 | 32.58 | 8.2 |  |
| GAIN 0 | 12/12 | 30.0 | 40.71 | 32.68 | 9.2 |  |
| GAIN 0.1 | 12/12 | 30.0 | 40.78 | 32.38 | 8.8 |  |
| GAIN 0.6 | 12/12 | 30.0 | 40.66 | 32.58 | 9.5 |  |
| GAIN 1 | 12/12 | 30.0 | 40.69 | 32.78 | 8.5 |  |
| GAIN 2 | 12/12 | 30.0 | 40.61 | 32.18 | 7.4 |  |
| LIMIT 0.05 rad | 12/12 | 30.0 | 40.80 | 32.48 | 9.4 |  |
| LIMIT 0.2 rad | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| LIMIT 0.6 rad | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |

### Speed of the last corner and of the finish

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| last corner 0.3 | 12/12 | 30.0 | 42.21 | 33.88 | 9.5 |  |
| last corner 0.6 | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| last corner 0.9 | 12/12 | 30.0 | 39.72 | 31.78 | 9.5 |  |
| last corner 1.28 | 6/12 | 28.5 | 41.56 | - | 9.5 | no finish in DONE x6 |
| finish 0.08 | 12/12 | 30.0 | 42.20 | 33.08 | 9.5 |  |
| finish 0.3 | 12/12 | 30.0 | 40.12 | 32.28 | 9.5 |  |
| finish 0.6 | 12/12 | 30.0 | 39.54 | 31.98 | 9.5 |  |

### Control latency (Pi/ROS between the strategy and the actuators)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| latency 0 ms | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| latency 20 ms | 12/12 | 30.0 | 41.14 | 33.08 | 6.9 |  |
| latency 50 ms | 12/12 | 30.0 | 41.45 | 33.38 | 7.6 |  |
| latency 100 ms | 12/12 | 30.0 | 41.89 | 33.88 | 3.0 |  |
| latency 200 ms | 5/12 | 14.7 | 39.64 | 33.88 | 0.0 | wall in TURN x7 |

### Motor: the speed the car really reaches (table scale; strategy keeps its table)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| real speed x0.5 | 12/12 | 30.0 | 76.12 | 60.08 | 5.6 |  |
| real speed x0.7 | 12/12 | 30.0 | 55.98 | 44.28 | 10.6 |  |
| real speed x0.85 | 12/12 | 30.0 | 47.03 | 37.18 | 12.1 |  |
| real speed x1 | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| real speed x1.15 | 12/12 | 30.0 | 35.99 | 28.58 | 7.0 |  |
| real speed x1.3 | 12/12 | 30.0 | 32.41 | 25.88 | 1.5 |  |
| real speed x1.5 | 2/12 | 14.6 | 25.53 | 22.78 | 0.0 | wall in TURN x8 |

### Motor: same, but the strategy table is also updated

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| real speed x0.7, table updated | 12/12 | 30.0 | 55.89 | 44.18 | 9.3 |  |
| real speed x0.85, table updated | 12/12 | 30.0 | 47.08 | 37.18 | 11.9 |  |
| real speed x1.15, table updated | 12/12 | 30.0 | 36.19 | 28.88 | 8.7 |  |
| real speed x1.3, table updated | 12/12 | 30.0 | 32.76 | 26.48 | 3.9 |  |

### Motor: slower response (drive_tau, drive_delay)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| drive_tau 0.05 s | 12/12 | 30.0 | 40.85 | 32.68 | 8.9 |  |
| drive_tau 0.15 s | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| drive_tau 0.3 s | 12/12 | 30.0 | 40.76 | 32.68 | 8.4 |  |
| drive_tau 0.6 s | 0/12 | 27.0 | - | - | 10.1 | no finish in DONE x12 |
| drive_delay 0 s | 12/12 | 30.0 | 40.80 | 32.78 | 8.6 |  |
| drive_delay 0.06 s | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| drive_delay 0.15 s | 12/12 | 30.0 | 40.79 | 32.78 | 8.8 |  |
| drive_delay 0.3 s | 12/12 | 30.0 | 40.68 | 32.48 | 8.6 |  |
| accel_max 0.3 m/s2 | 12/12 | 30.0 | 41.80 | 33.68 | 7.8 |  |
| accel_max 0.6 m/s2 | 12/12 | 30.0 | 41.06 | 32.98 | 8.4 |  |
| accel_max 2 m/s2 | 12/12 | 30.0 | 40.53 | 32.38 | 8.5 |  |

### Sensors: LiDAR dropout and range noise, gyroscope bias and noise

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| LiDAR dropout 0% | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| LiDAR dropout 4% | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| LiDAR dropout 8% | 12/12 | 30.0 | 40.73 | 32.58 | 9.1 |  |
| LiDAR dropout 12% | 12/12 | 30.0 | 40.67 | 32.58 | 9.1 |  |
| LiDAR dropout 15% | 12/12 | 30.0 | 40.71 | 32.58 | 7.8 |  |
| LiDAR dropout 25% | 2/12 | 13.2 | 32.23 | 32.38 | 0.0 | wall in TURN x10 |
| LiDAR dropout 40% | 2/12 | 7.0 | 31.53 | 31.98 | 0.0 | wall in TURN x7; wall in FOLLOW x3 |
| LiDAR range noise 0 mm | 12/12 | 30.0 | 40.78 | 32.58 | 9.4 |  |
| LiDAR range noise 8 mm | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| LiDAR range noise 20 mm | 10/12 | 25.8 | 40.72 | 32.58 | 0.0 | wall in TURN x2 |
| LiDAR range noise 40 mm | 1/12 | 6.2 | 30.68 | - | 0.0 | wall in TURN x7; wall in FOLLOW x3; stopped short in DONE x1 |
| LiDAR range noise 80 mm | 0/12 | 3.5 | - | - | 0.0 | wall in TURN x8; wall in FOLLOW x3; stopped short in DONE x1 |
| gyro bias 0 rad/s | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| gyro bias 0.006 rad/s | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| gyro bias 0.02 rad/s | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| gyro bias 0.05 rad/s | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| gyro bias 0.1 rad/s | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| gyro noise 0 rad/s | 12/12 | 30.0 | 40.73 | 32.48 | 9.3 |  |
| gyro noise 0.0019 rad/s | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| gyro noise 0.01 rad/s | 12/12 | 30.0 | 40.76 | 32.68 | 9.0 |  |
| gyro noise 0.03 rad/s | 12/12 | 30.0 | 40.78 | 33.08 | 8.7 |  |
| gyro noise 0.1 rad/s | 11/12 | 29.1 | 40.64 | 33.98 | 0.0 | wall in TURN x1 |

### Servo: slow servo (delay x3) with different fixes

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| delay x3, no change | 12/12 | 30.0 | 42.24 | 33.78 | 2.0 |  |
| delay x3, DAMPING 0 | 10/12 | 28.3 | 42.48 | 33.48 | 0.0 | wall in TURN x1 |
| delay x3, DAMPING 0.05 | 11/12 | 28.3 | 42.51 | 33.78 | 0.0 | wall in TURN x1 |
| delay x3, DAMPING 0.2 | 9/12 | 30.0 | 42.89 | 35.48 | 0.6 |  |
| delay x3, DAMPING 0.3 | 9/12 | 22.8 | 40.92 | 35.28 | 0.0 | wall in TURN x3 |
| delay x3, EXIT_ANGLE 0.1 | 12/12 | 30.0 | 42.24 | 33.78 | 2.0 |  |
| delay x3, EXIT_ANGLE 0.4 | 12/12 | 30.0 | 42.24 | 33.78 | 2.0 |  |
| delay x3, EXIT_ANGLE 0.6 | 12/12 | 30.0 | 42.24 | 33.78 | 2.0 |  |
| delay x3, TURN_FRONT 1.25 | 12/12 | 30.0 | 42.30 | 33.88 | 2.0 |  |
| delay x3, TURN_FRONT 1.3 | 12/12 | 30.0 | 42.31 | 33.78 | 2.0 |  |
| delay x3, TURN_FRONT 1.4 | 12/12 | 30.0 | 42.34 | 33.78 | 2.0 |  |
| delay x3, cruise 0.9 | 12/12 | 30.0 | 48.24 | 39.08 | 6.3 |  |
| delay x3, cruise 0.7 | 12/12 | 30.0 | 52.42 | 42.08 | 7.9 |  |

### Maximum steering angle: strategy and car agree (both change)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| both 0.2 rad (11.5 deg) | 7/12 | 22.0 | 40.68 | 34.98 | 0.0 | wall in TURN x4 |
| both 0.25 rad (14.3 deg) | 12/12 | 30.0 | 41.60 | 33.48 | 1.6 |  |
| both 0.28 rad (16.0 deg) | 12/12 | 30.0 | 41.13 | 32.88 | 5.4 |  |
| both 0.307 rad (17.6 deg) | 12/12 | 30.0 | 40.76 | 32.58 | 9.4 |  |
| both 0.35 rad (20.1 deg) | 12/12 | 30.0 | 40.35 | 31.98 | 9.2 |  |
| both 0.4 rad (22.9 deg) | 12/12 | 30.0 | 40.14 | 31.98 | 7.1 |  |
| both 0.4363 rad (25.0 deg) | 12/12 | 30.0 | 39.93 | 31.98 | 6.8 |  |

### Maximum steering angle: strategy assumes it but the car stays at 0.307

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| strategy assumes 0.2 rad (11.5 deg) | 7/12 | 21.2 | 39.24 | 34.18 | 0.0 | wall in TURN x5 |
| strategy assumes 0.25 rad (14.3 deg) | 12/12 | 30.0 | 41.10 | 33.08 | 4.2 |  |
| strategy assumes 0.35 rad (20.1 deg) | 12/12 | 30.0 | 40.76 | 32.58 | 9.5 |  |
| strategy assumes 0.4 rad (22.9 deg) | 12/12 | 30.0 | 40.76 | 32.58 | 9.5 |  |
| strategy assumes 0.4363 rad (25.0 deg) | 12/12 | 30.0 | 40.76 | 32.58 | 9.5 |  |

### Maximum steering angle: what the car does (strategy still assumes 0.307 rad)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| car reaches 0.2 rad (11.5 deg) | 7/12 | 22.0 | 40.73 | 35.38 | 0.0 | wall in TURN x4 |
| car reaches 0.25 rad (14.3 deg) | 12/12 | 30.0 | 41.60 | 33.48 | 1.6 |  |
| car reaches 0.28 rad (16.0 deg) | 12/12 | 30.0 | 41.16 | 33.18 | 5.5 |  |
| car reaches 0.307 rad (17.6 deg) | 12/12 | 30.0 | 40.76 | 32.58 | 9.4 |  |
| car reaches 0.35 rad (20.1 deg) | 12/12 | 30.0 | 40.41 | 31.88 | 7.1 |  |
| car reaches 0.4 rad (22.9 deg) | 12/12 | 30.0 | 40.41 | 31.88 | 7.1 |  |
| car reaches 0.4363 rad (25.0 deg) | 12/12 | 30.0 | 40.41 | 31.88 | 7.1 |  |

### Wheelbase: a longer or shorter car, code NOT updated vs updated

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| wheelbase 9.7 cm, strategy not updated | 12/12 | 30.0 | 40.36 | 31.98 | 6.0 |  |
| wheelbase 9.7 cm, WHEELBASE updated | 12/12 | 30.0 | 40.20 | 31.98 | 7.1 |  |
| wheelbase 11.7 cm, strategy not updated | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| wheelbase 14.7 cm, strategy not updated | 9/12 | 30.0 | 41.54 | 33.78 | 0.2 |  |
| wheelbase 14.7 cm, WHEELBASE updated | 9/12 | 30.0 | 41.62 | 33.68 | 0.1 |  |
| wheelbase 17.7 cm, strategy not updated | 7/12 | 19.5 | 39.65 | 34.48 | 0.0 | wall in TURN x5 |
| wheelbase 17.7 cm, WHEELBASE updated | 7/12 | 20.2 | 39.49 | 34.18 | 0.0 | wall in TURN x5 |
| wheelbase 23.7 cm, strategy not updated | 2/12 | 6.8 | 38.88 | 38.88 | 0.0 | wall in TURN x10 |
| wheelbase 23.7 cm, WHEELBASE updated | 2/12 | 6.8 | 36.98 | 36.98 | 0.0 | wall in TURN x10 |

### Gazebo rounds (`raw/gazebo.tsv`: name, direction, points, time from Start s, laps s, judge, extra)
```
baseline	ccw	30 de 30	34.42	11.0 / 10.5 / 10.7	sí   margen mínimo 9.5 cm	
baseline	cw	30 de 30	34.72	10.9 / 10.6 / 10.9	sí   margen mínimo 14.2 cm	
delay-x2	ccw	30 de 30	35.92	11.2 / 11.1 / 11.3	sí   margen mínimo 15.8 cm	actuator_params:=steer_delay=0.12
```
