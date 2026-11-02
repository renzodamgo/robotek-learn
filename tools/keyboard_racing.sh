#!/usr/bin/env bash
# Drive the real car like a racing game from this Mac: a small window, arrows / W A S D held and combined.
#
#   bash tools/keyboard_racing.sh [speed m/s] [steer deg]      # on the car first: car.launch.py teleop:=true
#
# Same network as the car (Cyclone DDS, ROS_DOMAIN_ID 1, discovery in the subnet, the Pi as a static peer).
# ROBOSTACK_ENV: the RoboStack env (default ~/robotek-ros/jazzy); PI_IP: the car (default 192.168.8.153).
cd "$(dirname "$0")/.." || exit 1
ARGS=("$@")   # keep speed and steer: macos_env.sh reads $1 as the env path
set --
source tools/macos_env.sh >/dev/null 2>&1 || { echo "No RoboStack env (ROBOSTACK_ENV=...)" >&2; exit 1; }
PI_IP=${PI_IP:-192.168.8.153}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=${REAL_CAR_DOMAIN_ID:-1} ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS=${ROS_STATIC_PEERS:-$PI_IP}
IFACE=$(route -n get "$PI_IP" 2>/dev/null | awk '/interface:/{print $2}')
[ -n "$IFACE" ] && export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$IFACE\" multicast=\"true\"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>"
echo "keyboard_racing: dominio $ROS_DOMAIN_ID, coche $PI_IP ${IFACE:+(interfaz $IFACE)}"
exec python3 tools/keyboard_racing.py "${ARGS[@]}"
