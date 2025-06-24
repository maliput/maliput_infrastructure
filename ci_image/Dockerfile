FROM ubuntu:20.04


ENV DEBIAN_FRONTEND noninteractive
ENV TERM linux
ENV LANG en_US.UTF-8
ENV ROS_DISTRO=foxy

##########################################################################
# Install base system dependencies
##########################################################################
RUN apt update && apt install -y sudo openssh-server software-properties-common \
   debian-keyring debian-archive-keyring apt-utils \
   locales && \
   rm -rf /var/lib/apt/lists/*
RUN locale-gen en_US.UTF-8
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections


##########################################################################
# Install basic tools
##########################################################################
RUN apt-get update && apt-get install -y \
    bash-completion \
    build-essential \
    git \
    lsb-release \
    wget \
    curl \
    cmake \
    python3 \
    python3-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

##########################################################################
# Install ros2 apt source
##########################################################################

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    apt install /tmp/ros2-apt-source.deb && \
    apt update && \
    rm -rf /tmp/ros2-apt-source.deb

##########################################################################
# Install utilities
##########################################################################

RUN apt install -y \
    python3-setuptools \
    python3-vcstool \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-ament-cmake \
    && rm -rf /var/lib/apt/lists/*

##########################################################################
# Final config
##########################################################################
RUN apt update && rosdep init && rosdep update --include-eol-distros

CMD ["/bin/bash"]
