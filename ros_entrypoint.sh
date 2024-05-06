#!/bin/bash
set -e

id -u ros &>/dev/null || adduser --quiet --disabled-password --gecos '' --uid ${UID:=1000} ros

getent group input || groupadd -g 107 input
usermod -aG input ros

source /opt/ros/${ROS_DISTRO}/setup.bash
spacenavd

exec "$@"
