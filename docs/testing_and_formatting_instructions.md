# Testing and Formatting Instructions

These are the instructions to run commands that format and test the ROS 2 packages in this repository.

Alternatively, once you enable the virtual environment, run this 1 bash script to rebuild, format and run tests:

1. `chmod +x rebuild_and_test.sh`
1. `./rebuild_and_test.sh`


## Formatting code

In a Terminal:

1. (Optional) Activate ROS 2 virtual environment: `source ~/ros2_venv/bin/activate`
1. Navigate to this repo's root directory
1. Run `clang-format` to format all source files
    - (Optional) to fix formatting in-place for 1 source file, just specify using the `i` flag (e.g `clang-format -i src/clock_pose_issuer/src/clock_pose_issuer.cpp`)

If you do not like the formatting style, just change `.clang-format` in the root of this repository.

## Running tests

In a Terminal:

1. (Optional) Activate ROS 2 virtual environment: `source ~/ros2_venv/bin/activate`
1. Navigate to this repo's root directory
1. Follow the ([Building and Sourcing Instructions](building_and_running_instructions.md#1-buildingsourcing-instructions))
1. Run the tests: `colcon test`
    - (Optional) If you only want to run a test for specific packages, just specify using the `packages-select` flag (e.g. `--packages-select clock_pose_issuer`)
1. View test results: `colcon test-result --verbose`