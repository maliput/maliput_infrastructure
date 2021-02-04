#Download base image ubuntu 18.04
FROM ubuntu:18.04

# Arguments for building
ARG USERID
ARG USER

# Install base system dependencies
RUN apt update && apt install -y sudo openssh-server software-properties-common \
   debian-keyring debian-archive-keyring apt-utils

# Setup environment
ENV TERM linux
ENV DEBIAN_FRONTEND noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
RUN apt install -y locales
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8

# Create a user with passwordless sudo
RUN adduser --uid $USERID --gecos "DSim Developer" --disabled-password $USER
RUN adduser $USER sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo "export QT_X11_NO_MITSHM=1" >> /home/$USER/.bashrc

USER $USER

COPY tools/install_dependencies.sh /home/$USER/

ARG WORKSPACE_NAME
WORKDIR /home/$USER/$WORKSPACE_NAME

CMD ["/bin/bash"]
