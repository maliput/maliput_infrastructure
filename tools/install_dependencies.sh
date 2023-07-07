#!/bin/bash
# Copyright 2021 Toyota Research Institute

set -e pipefail

#######################################
# Installs apt repository into system wide sources list.
#
# Arguments
#   $1 -> name of the repository, to be used as sources list prefix.
#   $2 -> url of the repository.
#######################################
install_apt_repo() {
    REPO_NAME=$1
    REPO_URL=$2
    KEY_PATH=/usr/share/keyrings/ros-archive-keyring.gpg

    if [ ! -f ${KEY_PATH} ]; then
        apt update
        apt install -y curl
        curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
             -o ${KEY_PATH}
    fi

    if ! grep -q "^deb .*$REPO_URL" /etc/apt/sources.list /etc/apt/sources.list.d/*; then
        echo "deb [signed-by=${KEY_PATH}] $REPO_URL $(lsb_release -cs) main" | \
        tee --append /etc/apt/sources.list.d/$REPO_NAME.list > /dev/null
        echo "Apt Repo '$REPO_NAME'..........................installed"
    else
        echo "Apt Repo '$REPO_NAME'..........................found"
    fi
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

# Get correspondant ROS DISTRO.
declare -A ROS_DISTRO_MAP
ROS_DISTRO_MAP[jammy]=humble
ROS_DISTRO_MAP[focal]=foxy
ROS_DISTRO=${ROS_DISTRO_MAP[$(lsb_release -sc)]}
# TODO: Exit if ROS DISTRO is unknown.

install_apt_repo "ros2-latest" "http://packages.ros.org/ros2/ubuntu"
apt update

# Get correspondant Clan version.
declare -A CLANG_SUITE_VERSION_MAP
CLANG_SUITE_VERSION_MAP[jammy]=14
CLANG_SUITE_VERSION_MAP[focal]=8
CLANG_SUITE_VERSION=${CLANG_SUITE_VERSION_MAP[$(lsb_release -sc)]}


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
