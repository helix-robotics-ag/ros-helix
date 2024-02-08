#!/bin/bash

REPOSITORY_NAME="$(basename "$(dirname -- "$( readlink -f -- "$0"; )")")"

docker run -it --rm \
--network=host \
--ipc=host \
--pid=host \
--env UID=${MY_UID} \
--env ROS_DOMAIN_ID \
--privileged \
--volume ./helix_bringup:/colcon_ws/src/helix_bringup \
--volume ./helix_description:/colcon_ws/src/helix_description \
--volume ./dynamixel_hardware:/colcon_ws/src/dynamixel_hardware \
ghcr.io/helix-robotics-ag/${REPOSITORY_NAME}:humble
