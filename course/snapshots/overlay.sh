#!/usr/bin/env bash
# Build the package my_open as it must be at step N in a scratch workspace and print its install prefix.
#
#   course/snapshots/overlay.sh N [WORKSPACE]      # default WORKSPACE: /tmp/ws_steps
#
# Run it from a shell where the ROS environment and the course workspace are already sourced
# (tools/macos_env.sh and install/setup.*). Then:  source WORKSPACE/install/setup.<shell>  and  python3 course/verify_step.py N
set -eu
step=${1:?usage: overlay.sh N [WORKSPACE]}
workspace=${2:-/tmp/ws_steps}
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../.." && pwd)
modules=(model heading perception lane arc turn_gate)       # step N has the first N of these

rm -rf "$workspace"
mkdir -p "$workspace/src"
cp -R "$root/src/my_open" "$workspace/src/my_open"
package="$workspace/src/my_open/my_open"
rm -rf "$package/__pycache__" "$package/strategy.py" "$package/mission.py" "$package/direction.py"
for i in "${!modules[@]}"; do
  if [ "$((i + 1))" -gt "$step" ]; then rm -f "$package/${modules[$i]}.py"; fi
done
cp "$here/drive_step$step.py" "$package/drive.py"
# setuptools < 80 is needed for --symlink-install on some macOS RoboStack environments (pin in $SETUPTOOLS_PIN)
cd "$workspace"
PYTHONPATH="${SETUPTOOLS_PIN:-}${SETUPTOOLS_PIN:+:}${PYTHONPATH:-}" colcon build --symlink-install --packages-select my_open
