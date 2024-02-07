#!/bin/bash
set -e

id -u ros &>/dev/null || adduser --quiet --disabled-password --gecos '' --uid ${UID:=1003} ros

source /opt/ros/${ROS_DISTRO}/setup.bash

exec "$@"
