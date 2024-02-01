FROM ros:humble

# Install necessary packages
RUN apt-get update && \
    apt-get install -y \
        g++ \
        cmake \
        git \
        libsocketcan-dev \
        can-utils \
        libeigen3-dev \
        gdb

# Create a new user with a specific UID and GID, and set up the workspace
RUN useradd -m -u 1000 -s /bin/bash nvidia && \
    mkdir -p /home/nvidia/starq_ws && \
    chown -R nvidia:nvidia /home/nvidia/starq_ws
WORKDIR /home/nvidia/starq_ws/src/

# Switch to the new non-root user
USER nvidia

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set the default command to execute when creating a new container
CMD ["bash"]
