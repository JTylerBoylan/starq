FROM ros:humble

# Install necessary packages
RUN apt-get update && \
    apt-get install -y \
        git wget unzip \
        g++ cmake gdb \
        libsocketcan-dev can-utils \
        libeigen3-dev \
        libx11-dev xorg-dev libglfw3 libglfw3-dev \
        ros-humble-joy \
        ros-humble-rviz2 \
        ros-humble-microstrain-inertial-driver \
        ros-humble-microstrain-inertial-examples \
        ros-humble-usb-cam \
        ros-humble-robot-localization

# Create a new user with a specific UID and GID, and set up the workspace
RUN useradd -m -u 1000 -s /bin/bash nvidia && \
    mkdir -p /home/nvidia/starq_ws && \
    chown -R nvidia:nvidia /home/nvidia/starq_ws

# Install MuJoCo
ENV MUJOCO_PATH /home/nvidia/MuJoCo
RUN git clone https://github.com/google-deepmind/mujoco ${MUJOCO_PATH}
RUN mkdir ${MUJOCO_PATH}/build && \ 
    cd ${MUJOCO_PATH}/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    cmake --build . --target install

# Install OSQP
ENV OSQP_PATH /home/nvidia/osqp
RUN git clone https://github.com/osqp/osqp ${OSQP_PATH}
RUN mkdir ${OSQP_PATH}/build && \
    cd ${OSQP_PATH}/build && \
    cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release .. && \
    cmake --build . --target install

# Give the user input group permissions
RUN groupadd -g 107 input && usermod -aG input nvidia

# Give the user sudo privileges
RUN echo 'nvidia:nvidia' | chpasswd
RUN echo 'nvidia ALL=(ALL) ALL' >> /etc/sudoers

# Switch to the new non-root user
USER nvidia

# Build project
COPY . /home/nvidia/starq_ws/src/
RUN cd /home/nvidia/starq_ws && \
    . /opt/ros/humble/setup.sh && \
    colcon build

RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib" >> ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "export PATH=$PATH:/home/nvidia/starq_ws/build/starq" >> ~/.bashrc

# Set the default command to execute when creating a new container
CMD ["bash"]
