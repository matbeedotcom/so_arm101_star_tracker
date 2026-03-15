#!/bin/bash

# Build script for ROS2 Humble Docker image on Raspberry Pi

echo "Building ROS2 Humble Docker image for Raspberry Pi..."

# Check if running on ARM64
ARCH=$(uname -m)
if [[ "$ARCH" != "aarch64" ]]; then
    echo "Warning: This Dockerfile is optimized for ARM64/aarch64 architecture."
    echo "Current architecture: $ARCH"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Build with docker-compose
docker-compose build

if [ $? -eq 0 ]; then
    echo "Docker image built successfully!"
    echo "You can now run the container with: ./docker_run.sh"
else
    echo "Docker build failed!"
    exit 1
fi