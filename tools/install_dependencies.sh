#!/bin/bash
# Copyright 2021 Toyota Research Institute

set -e pipefail

#######################################
# Pulls apt keys from a few well known keyservers.
#
# We've seen the "default" keyserver (p80.pool.sks-keyservers.net) fail somewhat
# often.  This function will try the default, followed by some alternates to
# reduce the number of times builds fail because of key problems.
#
# Arguments
#    $1 -> apt key
# Asserts
#    That apt key pulling was successful.
#######################################
pull_apt_keys() {
    success=0
    for keyserver in hkp://p80.pool.sks-keyservers.net:80 hkp://pgp.mit.edu:80 hkp://keyserver.ubuntu.com:80 ; do
        apt-key adv --keyserver $keyserver --recv-keys $1 || continue
        success=1
        return 0
    done
    return 1
}

#######################################
# Installs apt repository into system wide sources list.
#
# Arguments
#   $1 -> name of the repository, to be used as sources list prefix.
#   $2 -> url of the repository.
#   $3 -> apt key for the repository.
#######################################
install_apt_repo() {
    REPO_NAME=$1
    REPO_URL=$2
    REPO_KEY=$3

    if ! grep -q "^deb .*$REPO_URL" /etc/apt/sources.list /etc/apt/sources.list.d/*; then
        echo "deb $REPO_URL $(lsb_release -cs) main" | \
        tee --append /etc/apt/sources.list.d/$REPO_NAME.list > /dev/null
        pull_apt_keys $REPO_KEY
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

# Get correspondant ROS DISTRO.
declare -A ROS_DISTRO_MAP
ROS_DISTRO_MAP[bionic]=dashing
ROS_DISTRO_MAP[xenial]=bouncy
ROS_DISTRO=${ROS_DISTRO_MAP[$(lsb_release -sc)]}
# TODO: Exit if ROS DISTRO is unknown.

install_apt_repo "ros2-latest" "http://packages.ros.org/ros2/ubuntu" "C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"
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

# We initialize rosdep and discard the stdout message
# that recommends to run rosdep update.
rosdep init > /dev/null
