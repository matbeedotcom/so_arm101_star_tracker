#!/bin/bash

# Execute commands in the running ROS2 container

if [ $# -eq 0 ]; then
    # No arguments, attach interactive shell
    docker-compose exec ros2_humble bash
else
    # Execute the provided command
    docker-compose exec ros2_humble bash -c "source /opt/ros/humble/setup.bash && source /home/ros2_ws/install/setup.bash && $*"
fi