#!/bin/bash
# Copyright 2021 Toyota Research Institute

set -e

SCRIPT_FOLDER_PATH="$(cd "$(dirname "$0")"; pwd)"

show_help() {
  echo $'\nUsage: \t build.sh [OPTIONS] \n
  Options:\n
  \t-nv --nvidia\t       Image should have NVIDIA capabilities. \n
  \t-in --image_name\tName of the image to be built (default maliput_ws_ubuntu)\n
  \t-ws --workspace_folder\tName of the workspace folder (default maliput_ws)\n
  Example:\n
  \tbuild.sh --nvidia --image_name=custom_image_name \n'
}

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -nv|--nvidia) DOCKERFILE="Dockerfile.nvidia" ;;
        -in=*|--image_name=*) IMAGE_NAME="${1#*=}" ;;
        -ws=*|--workspace_folder=*) WORKSPACE_FOLDER="${1#*=}" ;;
        -h|--help) show_help ; exit 1 ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

echo "Building the docker image"
docker build -t ${IMAGE_NAME:-maliput_ws_ubuntu} \
     --file $SCRIPT_FOLDER_PATH/${DOCKERFILE:-Dockerfile} \
     --build-arg USERID=$(id -u) \
     --build-arg USER=$(whoami) \
     --build-arg WORKSPACE_FOLDER=${WORKSPACE_FOLDER:-maliput_ws} \
     $SCRIPT_FOLDER_PATH
