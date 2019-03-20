#!/usr/bin/env python

import argparse
import create_docker_files
import os
import pwd
import shutil
import subprocess
import sys

BASH_FILE_NAME = 'prepare_docker_environment'
DEPENDENCIES_SCRIPT_NAME = 'prepare_workspace'
DRAKE_BINARY_BASH_SCRIPT_FILE_NAME = 'get_drake_binary'

DOCKER_BUILD_BASH_SCRIPT = """#! /bin/bash
echo 'Building {DOCKER_IMAGE_NAME}'
docker build -t {DOCKER_IMAGE_NAME} -f {TEMPORARIES_PATH}/Dockerfile .;
cd {WORKSPACE_PATH};
{WORKSPACE_PATH}/start_ws.sh;
docker exec -ti `docker ps -q --filter ancestor={DOCKER_IMAGE_NAME}` sh -c "cd {WORKSPACE_PATH} && {TEMPORARIES_PATH}/{DEPENDENCIES_SCRIPT_NAME}";

echo 'Commiting docker image...'
docker commit `docker ps -q | head -n 1` {DOCKER_IMAGE_NAME};
docker stop {DOCKER_IMAGE_NAME};
"""

GET_DRAKE_BINARY_BASH_SCRIPT = """#! /bin/bash
RELEASE_URL="https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-bionic.tar.gz"

mkdir -p /opt/drake
echo 'Downloading drake...'
curl $RELEASE_URL | sudo tar xvz -C /opt/drake --strip 1
"""

DEPENDENCIES_SCRIPT = """#! /bin/bash
is_docker={IS_DOCKER}
mkdir -p {PATH_TO_WORKSPACE}/src
sudo apt-get update && sudo apt-get install -y sudo tmux git make python-vcstools openssh-server software-properties-common bash-completion debian-keyring debian-archive-keyring curl
git clone git@github.com:ToyotaResearchInstitute/dsim-repos-index.git {PATH_TO_WORKSPACE}/dsim-repos-index

echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable bionic main" | sudo tee --append /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-key adv --keyserver hkp://p80.pool.sks-keyservers.net:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
sudo apt-key adv --keyserver hkp://p80.pool.sks-keyservers.net:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116
echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" | sudo tee --append /etc/apt/sources.list.d/ros-latest.list > /dev/null
sudo apt-key adv --keyserver hkp://p80.pool.sks-keyservers.net:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

sudo apt-get update && sudo apt-get install -y python3 python3-setuptools python3-vcstool python3-pip
sudo apt-get update && sudo apt-get install -y python3-colcon-common-extensions
pip3 install --user pycodestyle
if [ -z {IGNITION_BINARY} ] && [ ! -z {DRAKE_BINARY} ]; then
    sudo apt install -y --no-install-recommends mercurial;
    vcs import {PATH_TO_WORKSPACE}/src < {PATH_TO_WORKSPACE}/dsim-repos-index/dsim-plus-ignition.repos;
elif [ -z {IGNITION_BINARY} ] && [ -z {DRAKE_BINARY} ]; then
    sudo apt install -y --no-install-recommends mercurial;
    vcs import {PATH_TO_WORKSPACE}/src < {PATH_TO_WORKSPACE}/dsim-repos-index/dsim-all.repos;
elif [ ! -z {IGNITION_BINARY} ] && [ -z {DRAKE_BINARY} ]; then
    sudo apt-get install -y libignition-gui0-dev libignition-rendering0-dev libignition-common2-dev libignition-tools-dev libignition-cmake1-dev libignition-math5-dev libignition-transport5-dev
    vcs import {PATH_TO_WORKSPACE}/src < {PATH_TO_WORKSPACE}/dsim-repos-index/dsim-plus-drake.repos;
else
    sudo apt-get install -y libignition-gui0-dev libignition-rendering0-dev libignition-common2-dev libignition-tools-dev libignition-cmake1-dev libignition-math5-dev libignition-transport5-dev
    vcs import {PATH_TO_WORKSPACE}/src < {PATH_TO_WORKSPACE}/dsim-repos-index/dsim-ci.repos;
fi
if [ ! -z {DRAKE_BINARY} ]; then
    sudo {TEMPORARIES_PATH}/get_drake_binary;
    yes | sudo /opt/drake/share/drake/setup/install_prereqs;
else
    sudo {PATH_TO_WORKSPACE}/src/drake/setup/ubuntu/install_prereqs.sh;
fi
sudo {PATH_TO_WORKSPACE}/src/delphyne_gui/tools/install_prereqs.sh
if [ "$is_docker" = false ]; then
    source {PATH_TO_WORKSPACE}/src/dsim-repos-index/setup/delphyne/env.bash
fi
"""

TEMPORARIES_PATH='{WORKSPACE_PATH}/.workspace'

def create_docker_bash_script(workspace_path, docker_file_name, use_drake_binary, temp_path):
    with open(os.path.join(temp_path, BASH_FILE_NAME), 'w') as bash_file:
        script = DOCKER_BUILD_BASH_SCRIPT.replace('{WORKSPACE_PATH}', workspace_path).replace(
            '{TEMPORARIES_PATH}', temp_path).replace(
                '{DEPENDENCIES_SCRIPT_NAME}', DEPENDENCIES_SCRIPT_NAME).replace(
                    '{DOCKER_IMAGE_NAME}', docker_file_name)
        bash_file.write(script)
        os.chmod(os.path.join(temp_path, BASH_FILE_NAME), 0775)
    workspace_shell_path = os.path.join(workspace_path, 'start_ws.sh')
    with open(workspace_shell_path, 'r') as shell_script:
        shell_script_data = shell_script.read()
    shell_script_data = shell_script_data.replace('-it', '-idt')
    with open(workspace_shell_path, 'w') as shell_script:
        shell_script.write(shell_script_data)
    shell_script_data = shell_script_data.replace('-idt', '-it')
    if use_drake_binary:
        get_drake_binary(temp_path)
    subprocess.call(os.path.join(temp_path, BASH_FILE_NAME), shell=True)
    with open(workspace_shell_path, 'w') as shell_script:
        shell_script.write(shell_script_data)

def get_drake_binary(temp_path):
    with open(os.path.join(temp_path,
        DRAKE_BINARY_BASH_SCRIPT_FILE_NAME), 'w') as bash_file:
        bash_file.write(GET_DRAKE_BINARY_BASH_SCRIPT)
        os.chmod(os.path.join(
            temp_path, DRAKE_BINARY_BASH_SCRIPT_FILE_NAME), 0775)

def create_workspace(path, use_drake_binary, use_ignition_source, is_docker, temp_path):
    if use_drake_binary:
        script = DEPENDENCIES_SCRIPT.replace('{DRAKE_BINARY}', "\"use_drake_binary\"")
        get_drake_binary(temp_path)
    else:
        script = DEPENDENCIES_SCRIPT.replace('{DRAKE_BINARY}', '\'\'')
    if use_ignition_source:
        script = script.replace('{IGNITION_BINARY}', '\'\'')
    else:
        script = script.replace('{IGNITION_BINARY}', "\"use_ignition_binary\"")
    if is_docker:
        script = script.replace('{IS_DOCKER}', "true")
    else:
        script = script.replace('{IS_DOCKER}', "false")
    script = script.replace('{PATH_TO_WORKSPACE}', path).replace(
        '{TEMPORARIES_PATH}', temp_path)
    with open(os.path.join(temp_path,
        DEPENDENCIES_SCRIPT_NAME), 'w') as bash_script:
        bash_script.write(script)
    os.chmod(os.path.join(
        temp_path, DEPENDENCIES_SCRIPT_NAME), 0775)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Create development enviroment.')
    parser.add_argument('-p', help='Path to workspace.'
        'Will use current dir by default', default=os.getcwd())
    parser.add_argument('-d', help='Docker image name.'
        'Will use delphyne:18.04 by default', default='delphyne:18.04')
    parser.add_argument('--use_ignition_source', help='Use ignition sources.'
        'Default value will be False.',
        action='store_true')
    parser.add_argument('--use_drake_binary', help='Use drake binary night release.'
        'Default value will be False.',
        action='store_true')
    parser.add_argument('--no_docker', help='Dont use docker for installation'
        'Default value will be False.', action='store_true')
    parser.add_argument('--force', help='Forces the creation of the path directory'
        'if it does not exist. Default is False', action='store_true')
    args = parser.parse_args()
    path = os.path.normpath(args.p)
    if not os.path.isdir(path):
        if args.force:
            os.makedirs(path)
        else:
            print('Dir {0} does not exist'.format(path))
            sys.exit()
    docker_file_name = args.d
    user_name = pwd.getpwuid(os.getuid())[0]
    modify_makefile = args.use_ignition_source or not args.use_drake_binary
    temp_path = TEMPORARIES_PATH.replace("{WORKSPACE_PATH}", path)
    if not os.path.isdir(temp_path):
        os.makedirs(temp_path)
    else:
        shutil.rmtree(temp_path)
        os.makedirs(temp_path)
    if args.no_docker:
        print("Creating no docker workspace")
        create_workspace(path, args.use_drake_binary, args.use_ignition_source, False, temp_path)

        subprocess.call(os.path.join(temp_path,
            DEPENDENCIES_SCRIPT_NAME), shell=True)
    else:
        print("Creating docker workspace")
        create_workspace(path, args.use_drake_binary, args.use_ignition_source, True, temp_path)
        create_docker_files.create_docker_file(
            user_name, path, docker_file_name, not args.use_ignition_source, args.use_drake_binary,
            temp_path)
        os.chdir(path)
        create_docker_bash_script(path, docker_file_name, args.use_drake_binary, temp_path)
