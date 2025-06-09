#!/bin/bash
# Copyright 2021 Toyota Research Institute

set -e pipefail

#######################################
# Installs ros2 apt source.
#######################################

install_ros2_apt_source() {
    apt update && apt install curl -y
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
    apt install /tmp/ros2-apt-source.deb
}

#######################################
# Installs basic workspace tools.
# Arguments:
#   Version of ros.
# Returns:
#   0 if no error was detected, non-zero otherwise.
#######################################
function install_workspace_tools() {
  local ros_distro=$1

  apt install -y \
              bash-completion \
              build-essential \
              curl \
              gdb \
              git \
              mercurial \
              python3  \
              python3-setuptools \
              python3-vcstool \
              python3-rosdep \
              python3-colcon-common-extensions \
              ros-${ros_distro}-ament-cmake \
              tmux
}

#######################################
# Installs clang suite packages.
# Arguments:
#   Version of the clang suite package.
# Returns:
#   0 if no error was detected, non-zero otherwise.
#######################################
function install_clang_suite() {
  local version=$1

  apt install -y \
              clang-${version} \
              lldb-${version} \
              lld-${version} \
              clang-format-${version} \
              clang-tidy-${version} \
              libc++-${version}-dev \
              libc++abi-${version}-dev
}

# In focal docker image, lsb_release is not available
apt update && apt install -y lsb-release

ROS_DISTRO=foxy

install_ros2_apt_source
apt update

# Define clang version.
CLANG_SUITE_VERSION=8

# Install dependencies.
install_workspace_tools ${ROS_DISTRO}
install_clang_suite ${CLANG_SUITE_VERSION}

# Source ros environments variables.
if ! grep -q "^source /opt/ros/$ROS_DISTRO/setup.bash" ~/.bashrc; then
    cat >> ~/.bashrc <<< "source /opt/ros/$ROS_DISTRO/setup.bash"
fi
if ! grep -q "^export ROS_DISTRO=" ~/.bashrc; then
    cat >> ~/.bashrc <<< "export ROS_DISTRO=$ROS_DISTRO"
fi
# Source colcon argcomplete
if ! grep -q "^source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ~/.bashrc; then
    cat >> ~/.bashrc <<< "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash"
fi

# We initialize rosdep and discard the stdout message
# that recommends to run rosdep update.
rosdep init > /dev/null
