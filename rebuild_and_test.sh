#!/bin/bash

# Exit immediately if any command fails
set -e

# Optional: enable debugging output
# set -x

echo "Cleaning workspace..."
rm -rf build/ install/ log/

# Optional: formatting
# echo "Formatting source files..."
# find src/ -type f \( -iname "*.cpp" -o -iname "*.hpp" \) -exec clang-format -i {} +

echo "Building packages..."
colcon build --symlink-install

echo "Sourcing environment..."
source install/setup.sh

echo "Running tests..."
export QT_QPA_PLATFORM=offscreen
colcon test

echo "Fetching test results..."
colcon test-result --verbose