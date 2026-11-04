#!/usr/bin/env bash
# One Open v5 test on the car WITHOUT the button, recording everything for analysis afterwards.
#
#   bash tools/open-debug-run.sh [extra launch arguments]      # e.g. motor_max_rps:=6 start_delay:=8
#
# Stops the autostart service (tools/car-autostart.sh) so the board and the LiDAR are free, starts a rosbag and
# launches car.launch.py with the Open v5, button:=false (the car starts by itself after start_delay, 12 s by
# default; the board button then STOPS it) and debug:=true. Ctrl-C ends the launch and closes the bag.
# Everything goes to ~/runs/<date>-<time>-open-debug/:
#   launch.log   the terminal output (debug: states, direction, corners, laps)
#   bag/         commands (/cmd_vel_raw), servo pulses and motor speed sent to the board, LiDAR, IMU,
#                battery, mission state, start/stop and steering arrow (5 s chunks)
#   cpu.log      top every second (8 busiest processes)
#   telemetry    link to the strategy's telemetry.csv (~/runs/<date>-<time>-open-v5/)
# The autostart is NOT turned back on: bash tools/car-autostart.sh start (or the next boot does it).
set -uo pipefail

REPO=$(cd "$(dirname "$0")/.." && pwd)
OUT=$HOME/runs/$(date +%Y%m%d-%H%M%S)-open-debug
mkdir -p "$OUT"

bash "$REPO/tools/car-autostart.sh" stop 2>/dev/null || true
# Nodes left over from an earlier launch (killed before its children stopped) would run next to the new ones: two
# strategies and two board drivers (4 nov 2026). Refuse to start until they are gone.
LEFT=$(ps -eo pid,args | grep -E "[r]obotek_hardware/lib|[r]obotek_course/lib|[r]rc_lite_driver/lib|[r]obotek_open/lib|[l]dlidar_stl_ros2/lib" | awk '{print $1}')
if [ -n "$LEFT" ]; then
    echo "Car nodes from an earlier launch are still running (PIDs: $(echo $LEFT)). Stop them first: kill -INT $(echo $LEFT)" >&2
    rmdir "$OUT" 2>/dev/null
    exit 1
fi
set +u
source /opt/ros/jazzy/setup.bash
source "$REPO/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=1 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# Not the detector JPEG nor /open/turn: the strategy only builds them while someone subscribes, and in its single thread
# that work slowed the control loop (33-39 Hz instead of 50, 4 nov 2026). RViz can still show them; DEBUG_BAG_ALL=1 adds them.
TOPICS=(/cmd_vel_raw /rrc/acker_servo/set_state /rrc/set_motor /robotek/scan /robotek/imu /rrc/imu_raw /rrc/battery
        /robotek/state /start /robotek/stop /robotek/estop /open/steer /joint_states /tf_static /rosout)
[ "${DEBUG_BAG_ALL:-0}" = 1 ] && TOPICS+=(/camera/direction_debug/compressed /open/turn)
ros2 bag record --max-bag-duration 5 -o "$OUT/bag" "${TOPICS[@]}" > "$OUT/bag.log" 2>&1 &
BAG=$!
# CPU per process every second (top), to see what slows the strategy.
top -b -d 1 -o %CPU -w 160 | grep --line-buffered -E "^top -|^ *[0-9]+ " | awk '/^top -/{print; n=0; next} n++<8' > "$OUT/cpu.log" &
TOP=$!

finish() {
    # A background job of a script may ignore SIGINT: fall back to SIGTERM (rosbag2 closes the bag on both).
    kill "$TOP" 2>/dev/null
    kill -INT "$BAG" 2>/dev/null
    for _ in $(seq 10); do kill -0 "$BAG" 2>/dev/null || break; sleep 0.5; done
    kill -TERM "$BAG" 2>/dev/null
    wait "$BAG" 2>/dev/null
    latest=$(ls -td "$HOME"/runs/*-open-v5 2>/dev/null | head -1)
    [ -n "$latest" ] && ln -sfn "$latest" "$OUT/telemetry"
    sync
    echo "Saved in $OUT"
}
trap finish EXIT

echo "Recording to $OUT. The car starts by itself after start_delay (12 s); the board button stops it; Ctrl-C ends."
ros2 launch robotek_course car.launch.py strategy:=robotek_open executable:=open_v5 button:=false debug:=true "$@" \
    2>&1 | tee "$OUT/launch.log"
