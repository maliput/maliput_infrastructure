#!/bin/bash

set -e

echo "Running the container..."

PROJECT_ROOT="$(cd "$(dirname "$0")"; cd ../..; pwd)" #Same level as dsim-repos-index

mkdir -p $PROJECT_ROOT/maliput_ws

exec docker run -it --rm \
       -e DISPLAY=$DISPLAY \
       -e SSH_AUTH_SOCK=$SSH_AUTH_SOCK \
       -v $(dirname $SSH_AUTH_SOCK):$(dirname $SSH_AUTH_SOCK) \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -v ${PROJECT_ROOT}/:/home/$(whoami) \
       -v /home/$USER/.ssh:/home/$USER/.ssh \
       --name maliput_ws_docker maliput_ws_ubuntu
