import argparse
import os
import pwd
import shutil
import sys

DOCKER_FILE = """FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu18.04

# Setup nvidia runtime
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Setup environment
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8

RUN apt-get update && apt-get install -y sudo tmux git make python-vcstools openssh-server software-properties-common bash-completion debian-keyring debian-archive-keyring curl

# Create a user with passwordless sudo
RUN adduser --gecos \"Development User\" --disabled-password {USER_NAME}
RUN adduser {USER_NAME} sudo
RUN echo \'%sudo ALL=(ALL) NOPASSWD:ALL\' >> /etc/sudoers
RUN echo \"export QT_X11_NO_MITSHM=1\" >> /home/{USER_NAME}/.bashrc
{DRAKE_ENV_VARS_FOR_BINARY}
RUN sudo mkdir -p /home/{USER_NAME}/.cache
RUN chown -R {USER_NAME} /home/{USER_NAME}/.cache

WORKDIR /home/{USER_NAME}
USER {USER_NAME}\n"""

SHELL_FILE = """#! /bin/bash
SCRIPT_PATH=$(dirname "$(readlink -f "$0")")
docker run --name {DOCKER_IMAGE_NAME} --privileged --rm --net=host -e DISPLAY=${DISPLAY} \\
       --runtime=nvidia -v $SCRIPT_PATH:$SCRIPT_PATH \\
       -v /var/cache/docker:/home/{USER_NAME}/.cache \\
       -v /home/{USER_NAME}/.ssh:/home/{USER_NAME}/.ssh \\
       -v /tmp/.X11-unix:/tmp/.X11-unix:ro $@ \\
       -it {DOCKER_IMAGE_NAME} /bin/bash
"""

IGNITION_BINARIES = ('libignition-gui0-dev '
'libignition-rendering0-dev '
'libignition-common2-dev '
'libignition-tools-dev '
'libignition-cmake1-dev '
'libignition-math5-dev '
'libignition-transport5-dev')

ENV_DRAKE_VARS_FOR_DOCKER_FILE = """ENV LD_LIBRARY_PATH=\"${LD_LIBRARY_PATH}:/opt/drake/lib\"
ENV PYTHONPATH=\"${PYTHONPATH}:/opt/drake/lib/python3.6/site-packages\"
ENV CMAKE_PREFIX_PATH=\"/opt/drake:${CMAKE_PREFIX_PATH}\"
ENV PATH=\"${PATH}:/home/{USER_NAME}/.local/bin\"
"""

def create_docker_file(user_name, path, docker_image_name, use_ign_binaries,
        use_drake_binary, docker_file_path):
    print('Using user: ' + user_name + " and path: " + path)
    docker_file = ''
    env_variables = ENV_DRAKE_VARS_FOR_DOCKER_FILE.replace(
        '{USER_NAME}', user_name)
    if use_ign_binaries:
        docker_file = DOCKER_FILE.replace(
            '{USER_NAME}', user_name).replace(
            '{USE_IGNITION_BINARIES}', IGNITION_BINARIES)
    else:
        docker_file = DOCKER_FILE.replace(
            '{USER_NAME}', user_name).replace(
            '{USE_IGNITION_BINARIES}', '')
    if use_drake_binary:
        docker_file = docker_file.replace(
            '{DRAKE_ENV_VARS_FOR_BINARY}', env_variables)
    else:
        docker_file = docker_file.replace(
            '{DRAKE_ENV_VARS_FOR_BINARY}', '')
    with open(os.path.join(docker_file_path, 'Dockerfile'), 'w') as file:
        file.write(docker_file)
    create_shell_file(user_name, path, docker_image_name)

def create_shell_file(user_name, path, docker_image_name):
    print('Creating script to start docker container...')
    with open(os.path.join(path, 'start_ws.sh'), 'w') as shell_file:
        sh_replaced = SHELL_FILE.replace('{USER_NAME}', user_name)
        shell_file.write(sh_replaced.replace('{DOCKER_IMAGE_NAME}', docker_image_name))
    os.chmod(os.path.join(path, 'start_ws.sh'), 0775)
