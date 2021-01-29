#!/bin/bash
# Copyright 2021 Toyota Research Institute

set -e

# Prints information about usage.
function show_help() {
  echo $'\nUsage: \t run.sh [OPTIONS] \n
  Options:\n
  \t-nv --nvidia\t        Selects nvidia runtime. \n
  \t-in --image_name\tName of the image to be run (default maliput_ws_ubuntu)\n
  \t-cn --container_name\tName of the container(default maliput_ws)\n
  \t-ws --workspace\tRelative or absolut path to the workspace you want to bind. (default to location of dsim-repos-index folder)\n
  Example:\n
  \trun.sh --nvidia --image_name=custom_image_name --container_name=custom_container_name \n
  \trun.sh --workspace=/path/to/your/ws_folder \n'
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
        -i=*|--image_name=*) IMAGE_NAME="${1#*=}" ;;
        -c=*|--container_name=*) CONTAINER_NAME="${1#*=}" ;;
        -w=*|--workspace=*) WORKSPACE="${1#*=}" ;;
        -h|--help) show_help ; exit 1 ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# Obtain effective workspace path.
WORKSPACE=${WORKSPACE:-${DSIM_REPOS_PARENT_FOLDER_PATH}/maliput_ws}
if is_relative_path $WORKSPACE ; then
  WORKSPACE="${RUN_LOCATION}/$WORKSPACE"
fi
WORKSPACE_FOLDER=$( basename $WORKSPACE )

xhost +
docker run -it --rm --runtime=$RUNTIME \
       -e DISPLAY=$DISPLAY \
       -e SSH_AUTH_SOCK=$SSH_AUTH_SOCK \
       -v $(dirname $SSH_AUTH_SOCK):$(dirname $SSH_AUTH_SOCK) \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -v ${WORKSPACE}/:/home/$(whoami)/$WORKSPACE_FOLDER/ \
       -v /home/$USER/.ssh:/home/$USER/.ssh \
       --name ${CONTAINER_NAME:-maliput_ws} ${IMAGE_NAME:-maliput_ws_ubuntu}
xhost -
