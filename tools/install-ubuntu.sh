#!/usr/bin/env bash
# Install ROS 2 Jazzy, Gazebo Harmonic (through ros_gz) and the tools of the course on Ubuntu 24.04 (also WSL2).
#
#   bash tools/install-ubuntu.sh
#
# It follows https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html and asks for sudo.
#
#   bash tools/install-ubuntu.sh --no-gazebo
#
# --no-gazebo (the Raspberry Pi of the real car, used by tools/setup-pi.sh): installs ros-jazzy-ros-base instead of
# the desktop (no RViz, no Gazebo, no ros_gz). Run as root (a container) it does not need sudo.
set -euo pipefail

GAZEBO=1
for arg in "$@"; do
  case "$arg" in
    --no-gazebo) GAZEBO=0 ;;
    -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
    *) echo "unknown option: $arg" >&2; exit 2 ;;
  esac
done
SUDO=sudo
if [ "$(id -u)" -eq 0 ]; then SUDO=; fi

echo ">>> [1/5] UTF-8 locale"
$SUDO apt update && $SUDO apt install -y locales
$SUDO locale-gen en_US en_US.UTF-8
$SUDO update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo ">>> [2/5] Ubuntu universe repository"
$SUDO apt install -y software-properties-common curl git
$SUDO add-apt-repository universe -y

echo ">>> [3/5] ROS 2 apt source"
$SUDO apt update
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")_all.deb"
$SUDO dpkg -i /tmp/ros2-apt-source.deb

if [ "$GAZEBO" = 1 ]; then
  echo ">>> [4/5] ROS 2 Jazzy desktop (RViz), Gazebo bridge, colcon and pytest (a few GB, take a coffee)"
  $SUDO apt update
  $SUDO apt install -y ros-dev-tools ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-rmw-cyclonedds-cpp python3-pytest
else
  echo ">>> [4/5] ROS 2 Jazzy base (no RViz, no Gazebo), colcon and pytest"
  $SUDO apt update
  $SUDO apt install -y ros-dev-tools ros-jazzy-ros-base ros-jazzy-rmw-cyclonedds-cpp python3-pytest
fi

echo ">>> [5/5] ROS 2 in every new terminal"
grep -qxF 'source /opt/ros/jazzy/setup.bash' ~/.bashrc || echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc

echo ">>> Check:"
bash -c 'source /opt/ros/jazzy/setup.bash && echo "ROS_DISTRO=$ROS_DISTRO"'
if [ "$GAZEBO" = 1 ]; then bash -c 'source /opt/ros/jazzy/setup.bash && gz sim --versions'; fi
echo ">>> Done. Open a new terminal."
