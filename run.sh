#!/bin/bash

REPOSITORY_NAME="$(basename "$(dirname -- "$( readlink -f -- "$0"; )")")"

docker run -it --rm \
--network=host \
--ipc=host \
--pid=host \
--env UID=$(id -u) \
--env GID=$(id -g) \
--privileged \
--volume ./helix_bringup:/colcon_ws/src/helix_bringup \
--volume ./helix_description:/colcon_ws/src/helix_description \
--volume ./dynamixel_hardware:/colcon_ws/src/dynamixel_hardware \
ghcr.io/helix-robotics-ag/${REPOSITORY_NAME}:iron
