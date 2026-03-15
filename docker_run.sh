#!/bin/bash

# Run script for ROS2 Humble Docker container

# Allow X11 forwarding (for GUI applications like RViz)
xhost +local:docker 2>/dev/null || true

# Function to clean up on exit
cleanup() {
    echo "Cleaning up..."
    xhost -local:docker 2>/dev/null || true
}
trap cleanup EXIT

# Check if the image exists
if ! docker images | grep -q "star_tracker_ros2.*humble"; then
    echo "Docker image not found. Building it first..."
    ./docker_build.sh
    if [ $? -ne 0 ]; then
        echo "Build failed. Exiting."
        exit 1
    fi
fi

# Run with docker-compose
echo "Starting ROS2 Humble container..."
docker-compose up -d

# Wait for container to start
sleep 2

# Attach to the container
docker exec -it star_tracker_ros2 bash

# Stop the container when exiting
read -p "Stop the container? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    docker-compose down
fi