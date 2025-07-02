# ROS2-Infrastructure
Robotics software infrastructure for a ROS2 test environment

**Prerequisite**: [ROS 2 Install Instructions](docs/install_instructions.md).

**Using the Packages**: [Building and Running Instructions](docs/building_and_running_instructions.md).


## Packages list

This repository has the following packages in the `src` directory:

1. **clock_pose_issuer**
    - Converts clock time to 6D poses on unit circle
1. **gui_pose_issuer**
    - GUI for user-desire manual 6D pose commands with an option for spacebar reset
1. **motion_controller**
    - Converts target poses to velocity commands

### Adding additional packages

TODO: update with instructions later

## Package Specifications

### Package connectivity

TODO: insert diagram here
- motion_controller decides which target pose to use
- `/cmd_vel` moves the turtle in sim

### Coordinate Systems

| Component              | Coordinate Frame     | Coordinate System Description                                    | Allowed Actions          | Responsible For                            |
| ---------------------- | -------------------- | ---------------------------------------------------------------- | ------------------------ | ------------------------------------------ |
| `motion_controller`    | World `[-1, 1]`      | Full normalized world frame centered at (0, 0)                   | Compute vx, vy           | High-level target pursuit in global map    |
| `pose_transformer`     | World `[-1, 1]`      | Maps turtlesim to `[-1, 1]`, but turtle only moves in radius 0.7 | Transform raw sim pose   | Convert `/turtle1/pose` → world-frame pose |
| `velocity_transformer` | Turtle’s local frame | Body frame; interprets vx, angular.z in robot coordinates        | linear.x, angular.z only | Convert world vx/vy → linear.x + angular.z |

### Topics list

| Node | Publishes (topics) | Subscribes (topics) |
| --- | --- | --- |
| clock_pose_issuer | `/target_pose_clock` | |
| gui_pose_issuer | `/target_pose_gui` | |
| motion_controller | `/cmd_vel` | `/target_pose_clock`, `/target_pose_gui`, `/sim_pose` |
