# Snapshots of `drive.py` per step

`drive_stepN.py` is the complete `my_open/drive.py` the reader has AT THE END of step N. Step N has only the modules
1 to N of this list (the final package, `src/my_open/my_open/`, has them all):

| step | modules present | what `drive` does |
|------|-----------------|-------------------|
| 1 | `model` | Drives a circle: 0.6 m/s commanded and a fixed steering (`steer` parameter, default 0.25 rad) after `/start`. Logs the radius `model.turn_radius` expects. No sensors. |
| 2 | + `heading` | Stands still 0.6 s after Start measuring the gyroscope bias (`calibrate` parameter, default true), then drives straight at 0.6 m/s holding heading 0 with `heading_steer`. |
| 3 | + `perception` | As 2, and for every distinct `/scan` computes `perceive(scan, 0, 1)`: publishes `/scene` (`Float32MultiArray`: left, right, front_axis, island_end) and `/scene_markers` (RViz, `robotek_course.viz.scene_markers`). Stops when `front_axis < 0.5`. |
| 4 | + `lane` | Follows the island wall (counter-clockwise, island on the left) at `GAP` 0.25 m with `lane_steer`, at 1.28 m/s commanded (0.6 m/s real). Stops when `front_axis <= 1.19`. |
| 5 | + `arc` | As 4, but at `front_axis <= 1.19` it runs exactly ONE 90 degree left corner with `arc_steer`, goes on along the lane and stops at the next wall. |
| 6 | + `turn_gate` | `TurnGate` (with the LiDAR odometry for `travelled`) decides every corner: it goes round and round, no mission, no laps, no stop. |

All of them: sim-time timer at 50 Hz, `geometry_msgs/Twist` on `/cmd_vel_raw` with `angular.z = speed * tan(steer) / WHEELBASE`,
the first `/start` (latched `Bool`, `TRANSIENT_LOCAL`) is the go signal. Run them with `-p use_sim_time:=true`.
The final `drive.py` (step 7 on) waits `CALIBRATE_SECONDS = 0.3`; these use 0.6 so that step 2 has 30 samples.

## The automatic check

```
source tools/macos_env.sh <env>; source install/setup.zsh          # ROS + course workspace, then the workspace with YOUR my_open
python3 course/verify_step.py N [--param steer:=0.2] [--compare] [--keep-logs]
```

`course/snapshots/overlay.sh N` builds `my_open` as it is at step N in `/tmp/ws_steps` (modules 1..N + `drive_stepN.py`),
which is how the snapshots were tested. `verify_step.py` starts `sim.launch.py gui:=false`, your `drive`, publishes `/start`
when the sensors are alive, records the TRUE pose (`/odom_gt`), ends everything and prints PASS/FAIL in Spanish.
Exit code 0 PASS, 1 FAIL, 2 could not run (another `gz sim` is running: use `--wait SECONDS`). It gives Gazebo a private
`GZ_PARTITION`, because Gazebo's transport ignores `ROS_DOMAIN_ID`.

Worlds (made from the package's SDF; `--world field|empty|lane` chooses, `--empty-world FILE` / `--lane-world FILE` write them):

* steps 1 and 2: **empty** (no walls, no mat). A circle does not fit in the 1 m corridor and 10 s of straight driving is longer than the field.
* step 4: **lane**: same corridor, but the island wall is 9 m long (no mat). The lane law closes a gap error with a time constant of
  about 5 s (3.3 m); the standard island is 1 m long, so from the spawn (gap 0.5 m) it only gets to ~0.43 m there.
* steps 3, 5, 6: the standard Open field.

Contact with a wall is detected from the true pose (car outline against the wall boxes read from the SDF, `Field.touches`); the
smallest distance between the outline and a wall is printed.

## What it measures, and what was measured (macOS, RoboStack, real_car sim, 2 to 4 runs each)

| step | PASS condition | measured |
|------|----------------|----------|
| 1 | fitted circle radius within +-4 % of `turn_radius(steer)` (steer 0.25, 14 s) | 0.463 m vs 0.460 m (+0.8 %), 3 of 3 runs. steer 0.1: 1.172 vs 1.170; 0.2: 0.582 vs 0.579. **steer 0.3 gives +12 to +13 %** (0.424 to 0.430 m vs 0.380 m): see below |
| 2 | true heading after 10 s (4 m) straight with calibration: error < 0.03 rad | -0.0012, +0.0045, -0.0021, +0.0000 rad. With `calibrate:=false` (`--compare`; not a pass condition): +0.060 and +0.053 rad (3 degrees) |
| 3 | `/scene` and `/scene_markers` published; `left`, `right`, `front_axis` within 5 cm and `island_end` within 10 cm of the numbers `perceive` gives on a perfect scan from the true pose (scan time +-0.06 s); stops with the wall at 0.10 to 0.50 m from the LiDAR; no contact. Not compared: scans of the braking phase (wall closer than 0.8 m) and the ones where the island's end is passing beside the car (the median jumps) | left 0.006 to 0.017 m, right 0.005 to 0.007 m, front 0.006 to 0.011 m, island_end 0.007 to 0.012 m; stops 0.14 m from the wall (bumper 0.12 m), nearest approach 0.117 m. 8 of 8 runs PASS after excluding those scans (before that the median at the island's end and the floor seen after braking made 1 run in 3 fail) |
| 4 | gap to the island at the stop order within [0.20, 0.31] m, never below 0.20 m in the second half, stopped before the wall ahead, no contact | gap 0.270, 0.241, 0.265 m after 8.7 m of lane (0.5 m at the start); stops 0.20 m from the wall (nearest approach 0.165 to 0.184 m) |
| 5 | after the first corner the true heading is pi/2 +-0.12 rad, in the east corridor, stopped at the next wall, no contact | pi/2 +0.019 and +0.018 rad; stops at about (0.80, 1.17); nearest approach 0.182 m |
| 6 | at least 8 corners (3 required) in 40 s, no contact | 14 corners (3.7 laps), 23.3 m, nearest approach 0.22 m, 3 of 3 runs |

## Things to know when writing the lessons

* **Speed**: `0.6` commanded is NOT 0.6 m/s: the motor table gives 0.42 m/s (`real_speed`). `1.28` commanded gives 0.6 m/s. Steps 1 to 3 use 0.6, steps 4 to 6 use 1.28.
* **Stopping takes distance**: the actuator model slews the speed command down at 0.6 m/s^2. From 1.28 the car needs about 1 m (step 4 stops 0.7 to 1 m after
  `front_axis <= 1.19`); from 0.6 about 0.35 m (step 3 stops 0.36 m after `front_axis < 0.5`, 0.14 m from the wall).
* **Steering above 0.278 rad saturates in Gazebo**: `sim_actuator_model` uses `wheelbase = 0.13` (its default, `sim.launch.py` does not pass 0.1174), so it
  scales the requested angle by 1.107 and clamps at 0.30718 rad. Up to steer 0.278 the radius is exactly `0.1174 / tan(steer)`; at full lock it is 0.42 to 0.43 m, not the 0.370 m
  of `model.turn_radius`. Passing `wheelbase: 0.1174` to `sim_actuator_model` in `sim.launch.py` would remove it (not done here).
* **The LiDAR sees the floor as a wall** when it is far or the car pitches: at about 5.1 m on the mat (3.8 m without it) the scan reads a constant range; after hard braking
  it can read 2.2 to 2.4 m. Nothing in the 3 x 3 m field is that far, so it only matters in a bigger world or when comparing `left` just after a stop.
* `ros2 run my_open drive` needs `--ros-args -p use_sim_time:=true` (the launch files pass it).
* The drive files end with `rclpy.try_shutdown()`: with `rclpy.shutdown()` after Ctrl-C, ROS 2 Jazzy can raise "rcl_shutdown already called".
