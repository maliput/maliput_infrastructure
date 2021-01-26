#!/bin/bash

set -e

CURRENT_FOLDER_PATH="$(cd "$(dirname "$0")"; pwd)"

echo "Building the docker image"
exec docker build -t maliput_ws_ubuntu \
     --build-arg USERID=$(id -u) \
     --build-arg USER=$(whoami) \
     $CURRENT_FOLDER_PATH
exec xhost +local:docker
