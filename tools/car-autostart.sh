#!/usr/bin/env bash
# Start the car's launch by itself every time the Raspberry Pi boots (systemd user service, no sudo).
#
#   bash tools/car-autostart.sh on [launch arguments]   # e.g. on motor_max_rps:=6 (default: the Open v5)
#   bash tools/car-autostart.sh off                     # stop it and do not start it at boot any more
#   bash tools/car-autostart.sh start | stop            # now, without changing the boot setting (both clean
#                                                       # leftover car nodes first: tools/car-clean.sh)
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
    # Nothing of an earlier launch may run next to this one (two strategies, two board drivers: 4 nov 2026).
    bash "$REPO/tools/car-clean.sh"
    # Every round leaves its evidence in ~/runs/<date>-<time>-autostart/ (4 nov 2026: a round that failed in its last
    # corner left nothing to explain it): launch.log (this output), kernel.log (USB drops of the board, "disabled by
    # hub (EMI?)") and bag/, a light rosbag in 5 s chunks that starts at Start (not while the car waits).
    local out=$HOME/runs/$(date +%Y%m%d-%H%M%S)-autostart
    mkdir -p "$out"
    journalctl -k -f -n 0 > "$out/kernel.log" 2>&1 &
    (ros2 topic echo --once /start > /dev/null 2>&1 &&
        exec ros2 bag record --max-bag-duration 5 -o "$out/bag" /cmd_vel_raw /rrc/acker_servo/set_state /rrc/set_motor \
            /rrc/reset /robotek/scan /robotek/imu /rrc/imu_raw /rrc/battery /robotek/state /start /robotek/stop \
            /robotek/estop /open/steer /tf_static /rosout > "$out/bag.log" 2>&1) &
    exec > >(tee "$out/launch.log") 2>&1
    echo "robotek-car: ros2 launch robotek_course car.launch.py ${args[*]} (recording to $out)"
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
    start | status)
        systemctl --user "$1" "$UNIT" ;;
    stop)
        # The launch can leave children behind: clean them too (tools/car-clean.sh).
        systemctl --user stop "$UNIT"
        bash "$REPO/tools/car-clean.sh" ;;
    log)
        journalctl --user -u "$UNIT" -b --no-pager | tail -n "${2:-60}" ;;
    run)
        run ;;
    *)
        sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'
        exit 1 ;;
esac
