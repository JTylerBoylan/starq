FROM ros:humble

# Install necessary packages
RUN apt-get update && \
    apt-get install -y \
        git wget unzip \
        g++ cmake gdb \
        libsocketcan-dev can-utils \
        libeigen3-dev \
        libx11-dev xorg-dev libglfw3 libglfw3-dev

# Create a new user with a specific UID and GID, and set up the workspace
RUN useradd -m -u 1000 -s /bin/bash nvidia && \
    mkdir -p /home/nvidia/starq_ws && \
    chown -R nvidia:nvidia /home/nvidia/starq_ws
WORKDIR /home/nvidia/starq_ws/src/

# Install MuJoCo
ENV MUJOCO_PATH /home/nvidia/MuJoCo
RUN git clone https://github.com/google-deepmind/mujoco ${MUJOCO_PATH}
RUN mkdir ${MUJOCO_PATH}/build && \ 
    cd ${MUJOCO_PATH}/build && \
    cmake .. && \
    cmake --build . --target install

# Install OSQP
ENV OSQP_PATH /home/nvidia/osqp
RUN git clone https://github.com/osqp/osqp ${OSQP_PATH}
RUN mkdir ${OSQP_PATH}/build && \
    cd ${OSQP_PATH}/build && \
    cmake -G "Unix Makefiles" .. && \
    cmake --build . --target install

# Install ORLQP
ENV ORLQP_PATH /home/nvidia/orlqp
ADD https://api.github.com/repos/JTylerBoylan/orlqp/git/refs/heads/main version.json
RUN git clone https://github.com/JTylerBoylan/orlqp ${ORLQP_PATH}
RUN mkdir ${ORLQP_PATH}/build && \
    cd ${ORLQP_PATH}/build && \
    cmake .. && \
    cmake --build . --target install
RUN ldconfig

# Switch to the new non-root user
USER nvidia

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set the default command to execute when creating a new container
CMD ["bash"]
