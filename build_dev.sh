#!/bin/bash

PROJECT_NAME=starq

# Get the absolute path to the directory containing this script
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Build the Docker image
docker build -t ${PROJECT_NAME}:latest "${SCRIPT_DIR}"