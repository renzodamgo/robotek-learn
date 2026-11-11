#!/usr/bin/env bash
# Launch a strategy on the car by hand, with everything it needs: stops the autostart and any leftover nodes
# (car-clean), sets the network the Mac sees (Cyclone DDS, domain 1, subnet discovery) and runs car.launch.
#
#   bash tools/car-run.sh open                      # Open v5 with the car defaults (robotek_open/config/car_picam.yaml)
#   bash tools/car-run.sh obstacle                  # Obstacle v3 (robotek_obstacle_v3/config/car_picam.yaml)
#   bash tools/car-run.sh open cruise_speed:=0.8    # any car.launch argument after the name
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
exec ros2 launch robotek_course car.launch.py "$@"
