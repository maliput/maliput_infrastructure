#!/bin/bash
# Copyright 2021 Toyota Research Institute

set -e

# Prints information about usage.
function show_help() {
  echo $'\nUsage:\t run.sh [OPTIONS] \n
  Options:\n
  \t-n --nvidia\t\t Selects nvidia runtime. \n
  \t-i --image_name\t\t Name of the image to be run (default maliput_ws_ubuntu)\n
  \t-c --container_name\t Name of the container(default maliput_ws)\n
  \t-w --workspace\t\t Relative or absolute path to the workspace you want to bind. (default to location of dsim-repos-index folder)\n
  Examples:\n
  \trun.sh --nvidia --image_name custom_image_name --container_name custom_container_name \n
  \trun.sh --workspace /path/to/your/ws_folder \n'
}

# Returns true when the path is relative, false otherwise.
#
# Arguments
#   $1 -> Path
function is_relative_path() {
  case $1 in
    /*) return 1 ;; # false
    *) return 0 ;;  # true
  esac
}

echo "Running the container..."

DSIM_REPOS_PARENT_FOLDER_PATH="$(cd "$(dirname "$0")"; cd ../..; pwd)"
# Location from where the script was executed.
RUN_LOCATION="$(pwd)"

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -n|--nvidia) RUNTIME="nvidia" ;;
        -i|--image_name) IMAGE_NAME="${2}"; shift ;;
        -c|--container_name) CONTAINER_NAME="${2}"; shift ;;
        -w|--workspace) WORKSPACE="${2}"; shift ;;
        -h|--help) show_help ; exit 1 ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# Use default if empty.
WORKSPACE=${WORKSPACE:-${DSIM_REPOS_PARENT_FOLDER_PATH}/maliput_ws}
# Convert into a absolute path if relative.
if is_relative_path $WORKSPACE ; then
  WORKSPACE="${RUN_LOCATION}/$WORKSPACE"
fi
WORKSPACE_FOLDER=$( basename $WORKSPACE )

# Update the arguments to default values if needed.
IMAGE_NAME=${IMAGE_NAME:-maliput_ws_ubuntu}
CONTAINER_NAME=${CONTAINER_NAME:-maliput_ws}

SSH_PATH=/home/$USER/.ssh
WORKSPACE_CONTAINER=/home/$(whoami)/$WORKSPACE_FOLDER/
SSH_AUTH_SOCK_USER=$SSH_AUTH_SOCK

xhost +
sudo docker run -it --rm --runtime=$RUNTIME \
       -e DISPLAY=$DISPLAY \
       -e SSH_AUTH_SOCK=$SSH_AUTH_SOCK_USER \
       -v $(dirname $SSH_AUTH_SOCK_USER):$(dirname $SSH_AUTH_SOCK_USER) \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -v ${WORKSPACE}:$WORKSPACE_CONTAINER \
       -v $SSH_PATH:$SSH_PATH \
       --name $CONTAINER_NAME $IMAGE_NAME
xhost -
