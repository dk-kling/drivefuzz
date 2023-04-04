#!/bin/bash

RUNTIME=""
DOCKER_VERSION=$(docker version --format '{{.Client.Version}}' | cut --delimiter=. --fields=1,2)
if [[ $DOCKER_VERSION < "19.03" ]] && ! type nvidia-docker; then
    RUNTIME="--gpus all"
else
    RUNTIME="--runtime=nvidia"
fi

XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority
SHDIR=/tmp/fuzzerdata

VOLUMES="--volume=$XSOCK:$XSOCK:rw --volume=$XAUTH:$XAUTH:rw --volume=$SHDIR:$SHDIR:rw"

docker run \
    -it --rm \
    --name="autoware-${USER}" \
    --volume=$(pwd)/autoware-contents:/home/autoware/autoware-contents:ro \
    --env="DISPLAY=${DISPLAY}" \
    --env="XAUTHORITY=${XAUTH}" \
    $VOLUMES \
    --privileged \
    --net=host \
    $RUNTIME \
    carla-autoware:improved ${TOWN} ${SP}
