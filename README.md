# ROS2-Infrastructure
Robotics software infrastructure for a ROS2 test environment

**Prerequisite**: [ROS 2 Install Instructions](docs/install_instructions.md).

**Using the Packages**: [Building and Running Instructions](docs/building_and_running_instructions.md).

**Testing and Formatting the Packages**: [Testing and Formatting Instructions](docs/testing_and_formatting_instructions.md).

## Packages list

This repository has the following packages in the `src` directory:

1. **clock_pose_issuer**
    - Converts clock time to 6D poses on unit circle
1. **gui_pose_issuer**
    - GUI for user-desire manual 6D pose commands with an option for spacebar reset
1. **motion_controller**
    - Converts target poses to velocity commands
1. **turtlesim_pose_transformer**
    - Transforms turtlesim poses, from the turtlesim's reference, to a unit circle reference and publishes those poses as the current pose
1. **turtlesim_velocity_transformer**
    - Transforms velocity commands, from a unit circle reference, to the turtlesim's reference and publishes those commands to turtlesim

### Adding additional packages

TODO: update with instructions later

## Package Specifications

### Package connectivity

![packages_topics_connectivity_system_diagram](docs/images/packages_topics_connectivity_system_diagram.png "ROS 2 Packages Connectivity Diagram")

### Coordinate Systems

| Component              | Coordinate Frame     | Coordinate System Description                                    | Allowed Actions          | Responsible For                            |
| ---------------------- | -------------------- | ---------------------------------------------------------------- | ------------------------ | ------------------------------------------ |
| `motion_controller`    | World `[-1, 1]`      | Full normalized world frame centered at (0, 0)                   | Compute vx, vy           | High-level target pursuit in global map    |
| `pose_transformer`     | World `[-1, 1]`      | Maps turtlesim to `[-1, 1]`, but turtle only moves in radius 0.7 | Transform raw sim pose   | Convert `/turtle1/pose` → world-frame pose |
| `velocity_transformer` | Turtle’s local frame | Body frame; interprets vx, angular.z in robot coordinates        | linear.x, angular.z only | Convert world vx/vy → linear.x + angular.z |

The turtle's local frame is about 11 by 11, where (0, 0) is in the bottom left corner and (11, 11) is in the top right

![unit_circle_analog_clock](docs/images/unit_circle_analog_clock.jpg "Unit circle on analog clock")


### Topics list

| Node | Publishes (topics) | Subscribes (topics) |
| --- | --- | --- |
| clock_pose_issuer | `/target_pose_clock` | |
| gui_pose_issuer | `/target_pose_gui` | |
| motion_controller | `/cmd_vel` | `/target_pose_clock`, `/target_pose_gui`, `/cur_pose` |
| turtlesim_pose_transformer | `/cur_pose` | `/turtle1/pose` |
| turtlesim_velocity_transformer | `/turtle1/cmd_vel` | `/cmd_vel`, `/cur_pose` |
