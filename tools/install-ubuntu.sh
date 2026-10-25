#!/usr/bin/env bash
# Install ROS 2 Jazzy, Gazebo Harmonic (through ros_gz) and the tools of the course on Ubuntu 24.04 (also WSL2).
#
#   bash tools/install-ubuntu.sh
#
# It follows https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html and asks for sudo.
set -euo pipefail

echo ">>> [1/5] UTF-8 locale"
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo ">>> [2/5] Ubuntu universe repository"
sudo apt install -y software-properties-common curl git
sudo add-apt-repository universe -y

echo ">>> [3/5] ROS 2 apt source"
sudo apt update
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

echo ">>> [4/5] ROS 2 Jazzy desktop (RViz), Gazebo bridge, colcon and pytest (a few GB, take a coffee)"
sudo apt update
sudo apt install -y ros-dev-tools ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-rmw-cyclonedds-cpp python3-pytest

echo ">>> [5/5] ROS 2 in every new terminal"
grep -qxF 'source /opt/ros/jazzy/setup.bash' ~/.bashrc || echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc

echo ">>> Check:"
bash -c 'source /opt/ros/jazzy/setup.bash && echo "ROS_DISTRO=$ROS_DISTRO" && gz sim --versions'
echo ">>> Done. Open a new terminal."
