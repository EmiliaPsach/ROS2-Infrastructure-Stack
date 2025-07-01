# Building and Running Instructions

These are the instructions to use the launch system for the ROS 2 packages in this repository.

First, build/source the ROS 2 packages anytime you want to change the content. Then, run the launch. Both sets of instructions are included below.

Alternatively, once you enable the virtual environment, run this 1 bash script to rebuild and run with the simulation:

1. `chmod +x rebuild_and_launch.sh`
1. `./rebuild_and_launch.sh`

## 1. Building/sourcing instructions

In a Terminal:

1. (Optional) Activate ROS 2 virtual environment: `source ~/ros2_venv/bin/activate`
1. Navigate to this repo's root directory
1. Clear your workspace before rebuilding to avoid any caching issues: `rm -rf build/ install/ log/`
1. Build the packages in this repository: `colcon build --symlink-install`
1. Set up your shell environment: `source install/setup.sh`

## 2. Running instructions

This repository flexibly enables a wide variety of launch modes for its ROS2 packages in `generate_launch_system.py`.

### Argument Table

Users can enable 0+ of these modes in combination:

| Argument | Default | Description |
| --- | --- | --- |
| `headless` | `false` | Disable GUI-based pose/setpoint input |
| `sim` | `false` | Enable simulation mode |
| `testing` | `false` | Enable testing mode |

If none of the optional flags are passed, then the default behavior is to enable all nodes.

### Example launch configurations

- **Default**: `ros2 launch src/robot_launch_system.py`
- **Simulation**: `ros2 launch src/robot_launch_system.py sim:=true`
