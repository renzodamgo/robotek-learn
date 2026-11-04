#!/usr/bin/env bash
# Start the car's launch by itself every time the Raspberry Pi boots (systemd user service, no sudo).
#
#   bash tools/car-autostart.sh on [launch arguments]   # e.g. on motor_max_rps:=6 (default: the Open v5)
#   bash tools/car-autostart.sh off                     # stop it and do not start it at boot any more
#   bash tools/car-autostart.sh start | stop            # now, without changing the boot setting
#   bash tools/car-autostart.sh status | log            # is it running, its output (journalctl)
#
# It always waits for the board button: button:=false is refused here, because the car would drive off
# 12 s after every boot. The arguments are saved in ~/.config/robotek/autostart-args; run "on" again to
# change them. It runs the robotek-learn of this folder (git pull + colcon build as usual; it uses the
# build at the next start). Needs lingering for the user (loginctl show-user $USER | grep Linger=yes) so it
# starts without anyone logging in; the Pi of the team already has it.
set -euo pipefail

REPO=$(cd "$(dirname "$0")/.." && pwd)
UNIT=robotek-car.service
UNIT_FILE=$HOME/.config/systemd/user/$UNIT
ARGS_FILE=$HOME/.config/robotek/autostart-args
DEFAULT_ARGS="strategy:=robotek_open executable:=open_v5"

run() {
    # Wait (up to 60 s) for an IPv4 address other than loopback: Cyclone DDS picks its network interface when it
    # starts, and without the Wi-Fi the Mac (RViz, keyboard) would not see the car.
    for _ in $(seq 60); do
        hostname -I 2>/dev/null | grep -q '[0-9]' && break
        sleep 1
    done
    set +u
    source /opt/ros/jazzy/setup.bash
    source "$REPO/install/setup.bash"
    set -u
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=1 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    read -r -a args < "$ARGS_FILE"
    echo "robotek-car: ros2 launch robotek_course car.launch.py ${args[*]}"
    exec ros2 launch robotek_course car.launch.py "${args[@]}"
}

case "${1:-}" in
    on)
        shift
        args=${*:-$DEFAULT_ARGS}
        case " $args " in
            *" button:=false "*|*" button:=False "*)
                echo "button:=false is not allowed at boot: the car would start driving by itself." >&2; exit 2 ;;
        esac
        case " $args " in *" strategy:="*) ;; *) args="$DEFAULT_ARGS $args" ;; esac
        mkdir -p "$(dirname "$UNIT_FILE")" "$(dirname "$ARGS_FILE")"
        echo "$args" > "$ARGS_FILE"
        cat > "$UNIT_FILE" <<EOF
[Unit]
Description=robotek car launch at boot (robotek-learn: car.launch.py, waits for the board button)

[Service]
Type=simple
ExecStart=/bin/bash $REPO/tools/car-autostart.sh run
# No restart: a car that relaunches by itself in the middle of a round is worse than one that stops.
Restart=no
KillSignal=SIGINT
TimeoutStopSec=15

[Install]
WantedBy=default.target
EOF
        systemctl --user daemon-reload
        systemctl --user enable "$UNIT"
        echo "At every boot: ros2 launch robotek_course car.launch.py $args"
        echo "Start it now with: bash tools/car-autostart.sh start"
        ;;
    off)
        systemctl --user disable --now "$UNIT" 2>/dev/null || true
        echo "Autostart off."
        ;;
    start | stop | status)
        systemctl --user "$1" "$UNIT" ;;
    log)
        journalctl --user -u "$UNIT" -b --no-pager | tail -n "${2:-60}" ;;
    run)
        run ;;
    *)
        sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'
        exit 1 ;;
esac
