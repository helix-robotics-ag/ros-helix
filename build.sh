#!/bin/bash

REPOSITORY_NAME="$(basename "$(dirname -- "$( readlink -f -- "$0"; )")")"

docker build --progress=plain -t ghcr.io/helix-robotics-ag/${REPOSITORY_NAME}:iron .