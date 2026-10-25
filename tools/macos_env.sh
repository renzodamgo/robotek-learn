# Prepare a terminal on macOS (RoboStack Jazzy) to build and run the course.
# Source it from the repo root, in zsh or bash:
#
#   source tools/macos_env.sh                 # default env: ~/robotek-ros/jazzy
#   source tools/macos_env.sh /path/to/env    # another RoboStack env
#

ROBOSTACK_ENV=${1:-${ROBOSTACK_ENV:-$HOME/robotek-ros/jazzy}}
if [ ! -x "$ROBOSTACK_ENV/bin/ros2" ]; then
  echo "macos_env: no RoboStack env with ros2 at $ROBOSTACK_ENV" >&2
  return 1 2>/dev/null || exit 1
fi
export ROBOSTACK_ENV
# RoboStack only: drop ROS/Gazebo variables inherited from Homebrew's ros2-<distro> or an old shell.
# A `colcon build` run with COLCON_PREFIX_PATH=/opt/homebrew/opt/ros2-jazzy chains that prefix into
# install/setup.sh, and `ros2 launch` then starts Homebrew's `create`/`parameter_bridge`, which abort.
unset COLCON_PREFIX_PATH ROS_PACKAGE_PATH PKG_CONFIG_PATH CMAKE_MODULE_PATH PYTHONPATH \
      LD_LIBRARY_PATH DYLD_LIBRARY_PATH GZ_CONFIG_PATH GZ_GUI_PLUGIN_PATH GZ_SIM_RESOURCE_PATH \
      GZ_SIM_SYSTEM_PLUGIN_PATH GZ_SIM_PHYSICS_ENGINE_PATH GZ_RENDERING_PLUGIN_PATH \
      GZ_RENDERING_RESOURCE_PATH GZ_SIM_RENDER_ENGINE_PATH
PATH=$(printf '%s' "$PATH" | tr ':' '\n' | grep -v '/opt/homebrew/opt/ros2-' | paste -sd: -)
export PATH="$ROBOSTACK_ENV/bin:$PATH"
# `micromamba activate` leaves AMENT_PREFIX_PATH empty: set it by hand.
export AMENT_PREFIX_PATH="$ROBOSTACK_ENV" CMAKE_PREFIX_PATH="$ROBOSTACK_ENV"
# The conda gz-rendering has a broken (space-padded) OGRE plugin path: without this the
# Metal render system is not found and `gz sim` crashes when the camera/LiDAR start.
export OGRE2_RESOURCE_PATH="$ROBOSTACK_ENV/lib/OGRE-Next"
export OGRE_RESOURCE_PATH="$ROBOSTACK_ENV/lib/OGRE"
# Fast DDS on macOS drops most 640x480 camera frames (UDP datagrams max 9216 bytes).
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Only on this machine: do not talk to other computers on the network.
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# CMake must use RoboStack's Python (not Homebrew's) to build the ament_cmake packages.
export COLCON_DEFAULTS_FILE="${TMPDIR:-/tmp}/robotek-colcon-defaults-macos.yaml"
cat >| "$COLCON_DEFAULTS_FILE" <<EOF_COLCON
build:
  cmake-args: ["-DPython3_EXECUTABLE=$ROBOSTACK_ENV/bin/python"]
EOF_COLCON

if [ -f install/setup.sh ] && grep -q '/opt/homebrew' install/setup.sh; then
  echo "macos_env: install/setup.sh chains Homebrew ROS; rebuild after sourcing this script (colcon build ...)" >&2
fi

if [ -f install/setup.bash ]; then
  if [ -n "${ZSH_VERSION:-}" ]; then source install/setup.zsh; else source install/setup.bash; fi
else
  echo "macos_env: no install/ yet; run: colcon build --symlink-install" >&2
fi

# Everything below must come from RoboStack (not Homebrew): ros2, gz, colcon.
for _tool in ros2 gz colcon; do
  case "$(command -v $_tool)" in
    "$ROBOSTACK_ENV"/*) ;;
    *) echo "macos_env: $_tool is not from $ROBOSTACK_ENV: $(command -v $_tool)" >&2 ;;
  esac
done
unset _tool
