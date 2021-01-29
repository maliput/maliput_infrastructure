#!/bin/bash
# Copyright 2021 Toyota Research Institute

set -e

# Prints information about usage.
function show_help() {
  echo $'\nUsage: \t build.sh [OPTIONS] \n
  Options:\n
  \t-nv --nvidia\t       Image should have NVIDIA capabilities. \n
  \t-in --image_name\tName of the image to be built (default maliput_ws_ubuntu)\n
  \t-ws --workspace_name\tName of the workspace folder (default maliput_ws)\n
  Example:\n
  \tbuild.sh --nvidia --image_name=custom_image_name --ws=maliput_ws \n'
}

echo "Building the docker image"

SCRIPT_FOLDER_PATH="$(cd "$(dirname "$0")"; pwd)"
DSIM_FOLDER_PATH="$(cd "$(dirname "$0")"; cd .. ; pwd)"
# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -n|--nvidia) DOCKERFILE="Dockerfile.nvidia" ;;
        -i=*|--image_name=*) IMAGE_NAME="${1#*=}" ;;
        -w=*|--workspace_name=*) WORKSPACE_NAME="${1#*=}" ;;
        -h|--help) show_help ; exit 1 ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

docker build -t ${IMAGE_NAME:-maliput_ws_ubuntu} \
     --file $SCRIPT_FOLDER_PATH/${DOCKERFILE:-Dockerfile} \
     --build-arg USERID=$(id -u) \
     --build-arg USER=$(whoami) \
     --build-arg WORKSPACE_NAME=${WORKSPACE_NAME:-maliput_ws} \
     $DSIM_FOLDER_PATH
