#!/bin/bash

xhost +local:root

PROJECT_NAME=starq
PROJECT_DIR=/home/nvidia/starq_ws/src/

# Get the absolute path to the directory containing this script
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Build the Docker image
docker build -t ${PROJECT_NAME}:latest "${SCRIPT_DIR}"

# Start the Docker container
docker run -it \
    --rm \
    --net host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "/${SCRIPT_DIR}:${PROJECT_DIR}" \
    ${PROJECT_NAME}:latest \
    bash
