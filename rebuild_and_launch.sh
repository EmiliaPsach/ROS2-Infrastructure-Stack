#!/bin/bash

# Exit immediately if any command fails
set -e

echo "Cleaning workspace..."
rm -rf build/ install/ log/

echo "Building packages..."
colcon build --symlink-install

echo "Sourcing environment..."
source install/setup.sh

echo "Launching simulation..."
ros2 launch src/robot_launch_system.py sim:=true
