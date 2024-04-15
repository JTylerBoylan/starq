#!/bin/bash

PROJECT_NAME=starq
PROJECT_DIR=/home/nvidia/starq_ws/src/

# Get the absolute path to the directory containing this script
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Allow root to connect to the X server
xhost +local:root

ARCH=$(uname -i)
if [ "$ARCH" == "aarch64" ]; then
    # Load the CAN interface
    sudo /bin/bash ${SCRIPT_DIR}/docs/loadcan_jetson.sh
fi

# Build the Docker image
docker build -t ${PROJECT_NAME}:latest "${SCRIPT_DIR}"

# Start the Docker container
docker run -it \
    --rm \
    --net host \
    --privileged \
    --name ${PROJECT_NAME} \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "/${SCRIPT_DIR}:${PROJECT_DIR}" \
    ${PROJECT_NAME}:latest \
    bash