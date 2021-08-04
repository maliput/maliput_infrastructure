#! /bin/bash

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
# Install ignition packages.
# Returns:
#   0 if no error was detected, non-zero otherwise.
#######################################
function install_ignition_packages() {

  install_apt_repo gazebo-stable \
        http://packages.osrfoundation.org/gazebo/ubuntu-stable \
        D2486D2DD83DB69272AFE98867170598AF249743

  apt update
  apt install -y \
    libignition-cmake2-dev \
    libignition-common3-dev \
    libignition-math6-dev \
    libignition-msgs5-dev \
    libignition-gui3-dev \
    libignition-rendering3-dev \
    libignition-tools-dev \
    libignition-transport8-dev
}

install_ignition_packages
