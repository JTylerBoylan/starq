#!/bin/bash

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
    -v "/${SCRIPT_DIR}:${PROJECT_DIR}" \
    ${PROJECT_NAME}:latest \
    bash
