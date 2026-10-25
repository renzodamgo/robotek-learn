#!/usr/bin/env bash
# One Gazebo round for the tuning experiments: cleans the simulator, runs the round without a window, prints one
# line (points, time, laps, margin) and appends it to raw/gazebo.tsv. Run it from the repository root with the
# RoboStack environment sourced (source tools/macos_env.sh; source install/setup.zsh).
#
#   bash course/maintainers/tuning/gazebo_round.sh NAME ccw|cw [launch arguments...]
#   bash course/maintainers/tuning/gazebo_round.sh slow-servo ccw actuator_params:=steer_delay=0.18
#   bash course/maintainers/tuning/gazebo_round.sh fast-last ccw 'set:=my_open.mission.LAST_CORNER_SPEED=1.1;my_open.mission.FINISH_SPEED=0.6'
#
# One round at a time: two Gazebo instances on one machine invalidate each other.
set -u
name=$1; direction=$2; shift 2
here=$(cd "$(dirname "$0")" && pwd)
log=${TMPDIR:-/tmp}/tuning-$name-$direction.log
bash src/robotek_evaluation/tools/clean_sim.sh >/dev/null 2>&1
perl -e 'alarm 300; exec @ARGV' ros2 launch robotek_course round.launch.py strategy:=my_open gui:=false \
    direction:="$direction" name:="$name" runs_dir:="${RUNS_DIR:-/tmp/tuning_runs}" "$@" >"$log" 2>&1
points=$(grep -m1 'Puntos:' "$log" | sed 's/.*Puntos: *//; s/ *(.*//')
time=$(grep -m1 'Tiempo desde Start' "$log" | sed 's/.*Start: *//; s/ s simulados.*//')
laps=$(grep -m1 'Tiempo desde Start' "$log" | sed 's/.*vueltas: *//; s/ *s)//')
valid=$(grep -m1 'Válida' "$log" | sed 's/.*Válida: *//')
printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "$name" "$direction" "${points:-none}" "${time:--}" "${laps:--}" "${valid:-no summary}" "$*" \
    | tee -a "$here/raw/gazebo.tsv"
bash src/robotek_evaluation/tools/clean_sim.sh >/dev/null 2>&1
left=$(ps ax -o command | grep -E 'gz si[m]|parameter_bridg[e]|round_recorde[r]|sim_actuator_mode[l]' | wc -l)
[ "$left" -eq 0 ] || echo "WARNING: $left leftover simulator processes"
