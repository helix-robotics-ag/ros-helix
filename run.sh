#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

export HOST_UID=$(id -u)

docker compose -f $SCRIPT_DIR/docker-compose.yml run \
--volume $SCRIPT_DIR/helix_bringup:/colcon_ws/src/helix_bringup \
--volume $SCRIPT_DIR/helix_description:/colcon_ws/src/helix_description \
--volume $SCRIPT_DIR/dynamixel_hardware:/colcon_ws/src/dynamixel_hardware \
ros-helix bash
