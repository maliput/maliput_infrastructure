#!/bin/bash
# Copyright 2021 Toyota Research Institute

set -e

echo "Running the container..."

PROJECT_ROOT="$(cd "$(dirname "$0")"; cd ../..; pwd)" #Same level as dsim-repos-index

show_help() {
  echo $'\nUsage: \t run.sh [OPTIONS] \n
  Options:\n
  \t-nv --nvidia\t        Selects nvidia runtime. \n
  \t-in --image_name\tName of the image to be run (default maliput_ws_ubuntu)\n
  \t-cn --container_name\tName of the container(default maliput_ws)\n
  Example:\n
  \trun.sh --nvidia --image_name=custom_image_name --container_name=custom_container_name \n'
}

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -nv|--nvidia) RUNTIME="nvidia" ;;
        -in=*|--image_name=*) IMAGE_NAME="${1#*=}" ;;
        -cn=*|--container_name=*) CONTAINER_NAME="${1#*=}" ;;
        -h|--help) show_help ; exit 1 ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

xhost +
docker run -it --rm --runtime=$RUNTIME \
       -e DISPLAY=$DISPLAY \
       -e SSH_AUTH_SOCK=$SSH_AUTH_SOCK \
       -v $(dirname $SSH_AUTH_SOCK):$(dirname $SSH_AUTH_SOCK) \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -v ${PROJECT_ROOT}/:/home/$(whoami) \
       -v /home/$USER/.ssh:/home/$USER/.ssh \
       --name ${CONTAINER_NAME:-maliput_ws} ${IMAGE_NAME:-maliput_ws_ubuntu}
xhost -
