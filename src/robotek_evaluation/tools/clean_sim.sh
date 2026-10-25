#!/usr/bin/env bash
# Kill every Robotek sim/strategy process, including orphaned nodes left behind
# when a launch is killed. Run this before every launch.
#
#   bash src/robotek_evaluation/tools/clean_sim.sh
#
# (patterns use a bracket so this script's own command line never matches.)
set -u

patterns=(
  'ros2 laun[c]h'
  '(^|[ /])parameter_bridge([ ]|$)'
  '(^|[ /])drive([ ]|$)'
  '(^|[ /])(contact_recorde[r]|round_recorde[r]|sim_actuator_mode[l]|sim_scan_mode[l])([ ]|$)'
  '(^|[ /])robot_state_publisher([ ]|$)'
  'gz si[m]'
  'gz-sim'
  '(^|[ /])rviz2([ ]|$)'
)

# Never signal this script or its callers: an agent/terminal command line can
# contain "ros2 launch" or "gz sim" and would otherwise be killed by -f matching.
ancestors=" "
pid=$$
while [ -n "$pid" ] && [ "$pid" -gt 1 ]; do
  ancestors+="$pid "
  pid=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
done

matching() {
  for pat in "${patterns[@]}"; do
    for p in $(pgrep -f "$pat" 2>/dev/null); do
      case "$ancestors" in *" $p "*) ;; *) echo "$p" ;; esac
    done
  done | sort -u
}

pids=$(matching); [ -n "$pids" ] && kill $pids 2>/dev/null
sleep 2
# second pass: force-kill anything that ignored the first (e.g. gz sim server/gui)
pids=$(matching); [ -n "$pids" ] && kill -9 $pids 2>/dev/null
sleep 1

leftover=$(for p in $(matching); do ps -o pid=,args= -p "$p" 2>/dev/null; done)

if [ -z "$leftover" ]; then
  echo "clean: no sim/strategy processes running"
else
  echo "still running:"; echo "$leftover"
fi
