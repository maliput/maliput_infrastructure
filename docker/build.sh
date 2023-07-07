#!/bin/bash
# Copyright 2023 Toyota Research Institute

set -e

# Prints information about usage.
function show_help() {
  echo $'\nUsage:\t build.sh [OPTIONS] \n
  Options:\n
  \t-n --nvidia\t\t Image should have NVIDIA capabilities.\n
  \t-o --os_version\t\t Name of the OS Version (default focal).\n
  \t-i --image_name\t\t Name of the image to be built (default maliput_ws_ubuntu_{OS_VERSION}).\n
  \t-w --workspace_name\t Name of the workspace folder (default is maliput_ws).\n
  Example:\n
  \tbuild.sh --nvidia --image_name custom_image_name --workspace_name maliput_ws \n'
}

echo "Building the docker image"

SCRIPT_FOLDER_PATH="$(cd "$(dirname "$0")"; pwd)"
DSIM_FOLDER_PATH="$(cd "$(dirname "$0")"; cd .. ; pwd)"
# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -n|--nvidia) DOCKERFILE="Dockerfile.nvidia" ;;
        -o|--os_vesrion) OS_VERSION="${2}"; shift ;;
        -i|--image_name) IMAGE_NAME="${2}"; shift ;;
        -w|--workspace_name) WORKSPACE_NAME="${2}"; shift ;;
        -h|--help) show_help ; exit 1 ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

OS_VERSION=${OS_VERSION:-focal}
IMAGE_NAME=${IMAGE_NAME:-maliput_ws_ubuntu_${OS_VERSION}}
DOCKERFILE_PATH=$SCRIPT_FOLDER_PATH/${OS_VERSION}/${DOCKERFILE:-Dockerfile}

WORKSPACE_NAME=${WORKSPACE_NAME:-maliput_ws}

USERID=$(id -u)
USER=$(whoami)

sudo docker build -t $IMAGE_NAME \
     --file $DOCKERFILE_PATH \
     --build-arg USERID=$USERID \
     --build-arg USER=$USER \
     --build-arg WORKSPACE_NAME=$WORKSPACE_NAME \
     $DSIM_FOLDER_PATH
