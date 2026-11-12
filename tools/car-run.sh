#!/usr/bin/env bash
# Launch a strategy on the car by hand, with everything it needs: stops the autostart and any leftover nodes
# (car-clean), sets the network the Mac sees (Cyclone DDS, domain 1, subnet discovery) and runs car.launch.
#
#   bash tools/car-run.sh open                      # Open v5 with the car defaults (robotek_open/config/car_picam.yaml)
#   bash tools/car-run.sh obstacle                  # Obstacle v3 (robotek_obstacle_v3/config/car_picam.yaml)
#   bash tools/car-run.sh open cruise_speed:=0.8    # any car.launch argument after the name
#   bash tools/car-run.sh open camera_always:=true  # the camera on for the whole round, to watch or record it in RViz
#   VERBOSE=1 bash tools/car-run.sh open             # every log line (default: only the summary below)
#
# Shown by default: ready / running, Start and Stop, the strategy's state changes with the time since Start, lap times,
# the final time, and anything that looks wrong (errors, a dead node, board reboots, low battery). The whole output is
# still in ~/.ros/log/<date>/launch.log.
#
# The car's defaults (picam camera, rear sensor only for the Obstacle, motor cap, speeds) come from car.launch and the
# strategy's car_picam.yaml: nothing else to type.
set -eo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
case "${1:-}" in
    open) shift; set -- strategy:=robotek_open executable:=open_v5 "$@" ;;
    obstacle) shift; set -- strategy:=robotek_obstacle_v3 "$@" ;;
    *) echo "usage: bash tools/car-run.sh open|obstacle [car.launch args...]" >&2; exit 2 ;;
esac
bash "$REPO/tools/car-autostart.sh" stop >/dev/null 2>&1 || true
bash "$REPO/tools/car-clean.sh"
source /opt/ros/jazzy/setup.bash
source "$REPO/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=1 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
if [ "${VERBOSE:-0}" = 1 ]; then
    exec ros2 launch robotek_course car.launch.py "$@"
fi
ros2 launch robotek_course car.launch.py "$@" 2>&1 | grep --line-buffered -E \
    'READY|RUNNING|: Start\.|: Stop|state [A-Z_]+|LAP [0-9]|DONE:|CRASH|ERROR|Traceback|has died|rebooted|Board reset|Battery [0-9.]+ V <'
