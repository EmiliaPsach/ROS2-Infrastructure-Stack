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

## Package connectivity

### Diagram

TODO: insert diagram here
- motion_controller decides which target pose to use
- `/cmd_vel` moves the turtle in sim

### Topics list

| Node | Publishes (topics) | Subscribes (topics) |
| --- | --- | --- |
| clock_pose_issuer | `/target_pose_clock` | |
| gui_pose_issuer | `/target_pose_gui` | |
| motion_controller | `/cmd_vel` | `/target_pose_clock`, `/target_pose_gui` |


#### Testing topic publishing

To test whether the topics are publishing correctly,

1. Follow the [Building and Running Instructions](docs/building_and_running_instructions.md)
1. Open a new terminal
    1. Livestream the message output: `ros2 topic echo <insert message name here>`

**Example output**:

```yaml
ros2 topic echo /target_pose_clock
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: map
pose:
  position:
    x: 0.7874430662013701
    y: -0.6163873923851662
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.9453684641983172
    w: 0.3260037835659504
---
```
