# Multi-stage build for ROS2 Humble on Raspberry Pi (ARM64)
FROM arm64v8/ubuntu:22.04 as base

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install locale
RUN apt-get update && \
    apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8

# Setup timezone
ENV TZ=UTC
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install ROS2 Humble dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    software-properties-common \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Add ROS2 apt repository
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# Install additional ROS2 packages for MoveIt and robot control
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-ros-perception \
    ros-humble-moveit-servo \
    ros-humble-moveit-setup-assistant \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-ros-control-interface \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-diagnostic-updater \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    && rm -rf /var/lib/apt/lists/*

# Install development tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-vcstool \
    python3-pip \
    python3-numpy \
    python3-opencv \
    vim \
    nano \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir \
    setuptools==58.2.0 \
    transforms3d \
    numpy \
    opencv-python

# Initialize rosdep
RUN rosdep init && \
    rosdep update

# Create workspace
WORKDIR /home/ros2_ws

# Copy the workspace
COPY ./src ./src

# Install dependencies using rosdep
RUN . /opt/ros/humble/setup.sh && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Create entrypoint script
RUN echo '#!/bin/bash' > /ros_entrypoint.sh && \
    echo 'set -e' >> /ros_entrypoint.sh && \
    echo '' >> /ros_entrypoint.sh && \
    echo '# Setup ROS2 environment' >> /ros_entrypoint.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /ros_entrypoint.sh && \
    echo 'source /home/ros2_ws/install/setup.bash' >> /ros_entrypoint.sh && \
    echo '' >> /ros_entrypoint.sh && \
    echo '# Execute the command passed to docker run' >> /ros_entrypoint.sh && \
    echo 'exec "$@"' >> /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# Expose ROS2 default ports
EXPOSE 11311
EXPOSE 8080