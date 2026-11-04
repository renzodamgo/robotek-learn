#!/usr/bin/env bash
# Stop every car node still running from an earlier launch, then set the motors to 0.
#
#   bash tools/car-clean.sh            # before each Open (car-autostart.sh and open-debug-run.sh call it)
#
# A launch stopped from a script or over ssh can leave its children running (a background job ignores Ctrl-C). On
# 4 nov 2026 such leftovers ran a second strategy and a second board driver next to the new launch, and three
# round_reporter at 22-30 % CPU each slowed the strategy to 4-14 Hz: the car drove straight into a wall.
# It stops: the nodes of this workspace (install/*/lib), the ROS helpers car.launch.py starts (robot_state_publisher,
# static_transform_publisher, usb_cam), ros2 launch / ros2 bag record of a car run and the debug recorder.
# SIGINT first (nodes close cleanly), then SIGTERM, then SIGKILL for whatever is still there.
set -uo pipefail

REPO=$(cd "$(dirname "$0")/.." && pwd)
# Bracketed first letters: the grep does not match its own command line.
PATTERN="[$(echo "${REPO:0:1}")]${REPO:1}/install/[^ ]*/lib/|[o]pt/ros/jazzy/lib/robot_state_publisher/|[o]pt/ros/jazzy/lib/tf2_ros/static_transform_publisher|[u]sb_cam_node|[r]os2 launch robotek_course|[r]os2 bag record|[o]pen-debug-run.sh|[t]op -b -d 1 -o %CPU"

pids() { ps -eo pid=,args= | grep -E "$PATTERN" | awk -v me=$$ -v parent=$PPID '$1 != me && $1 != parent {print $1}'; }

found=$(pids | tr '\n' ' ')
if [ -z "${found// }" ]; then
    echo "car-clean: no car nodes running."
else
    echo "car-clean: stopping $(echo $found | wc -w) processes: $found"
    for signal in INT TERM KILL; do
        left=$(pids | tr '\n' ' ')
        [ -z "${left// }" ] && break
        kill -"$signal" $left 2>/dev/null
        for _ in $(seq 10); do [ -z "$(pids)" ] && break; sleep 0.5; done
    done
    left=$(pids | tr '\n' ' ')
    [ -n "${left// }" ] && echo "car-clean: still running after SIGKILL: $left" >&2
fi

# Motors to 0 straight on the board (nothing else holds the port now). Skipped when there is no board.
if [ -e /dev/rrc_lite ]; then
    timeout 5 python3 - "$REPO" <<'PY' 2>/dev/null && echo "car-clean: motors at 0."
import sys, time
sys.path.insert(0, sys.argv[1] + '/src/rrc_lite_driver/rrc_lite_driver')
from rrc_lite_driver_sdk.board import Board
board = Board(device='/dev/rrc_lite')
for _ in range(10):
    board.set_motor_speed([[1, 0.0], [2, 0.0], [3, 0.0], [4, 0.0]])
    time.sleep(0.05)
PY
fi
exit 0
