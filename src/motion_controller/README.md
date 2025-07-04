# Motion Controller

The `motion_controller` ROS 2 package controls the robot's movement by following target poses issued from two different sources:

- `clock_pose_issuer` (automatic pose on a unit circle based on clock time)
- `gui_pose_issuer` (manual GUI-driven pose input)

This node subscribes to target poses from both sources and to the robot’s current pose, computing velocity commands to move the robot accordingly. It also listens to a boolean flag `/clock_mode_enabled` that toggles between automatic clock mode and manual GUI control.

---

## Topics

| Topic Name           | Message Type                    | Description                              |
|----------------------|--------------------------------|------------------------------------------|
| `/cmd_vel`           | `geometry_msgs/msg/Twist`      | Publishes computed velocity commands to control the robot’s movement |
| `/target_pose_clock`  | `geometry_msgs/msg/PoseStamped`| Receives clock-based target poses         |
| `/target_pose_gui`    | `geometry_msgs/msg/PoseStamped`| Receives GUI-based target poses           |
| `/cur_pose`           | `geometry_msgs/msg/PoseStamped`| Receives current robot pose               |
| `/clock_mode_enabled` | `std_msgs/msg/Bool`            | Receives mode flag: `true` = clock mode enabled, `false` = GUI mode |

---

## Parameters

- None configurable directly in the shown code, but topics and other behavior can be extended via ROS 2 parameters.

---

## Behavior and Target Pose Selection Logic

- The controller listens to target poses from both the clock and GUI pose issuers.
- It also listens to `/clock_mode_enabled` to determine the active control mode.
- If clock mode is enabled (`true`), the robot follows the clock pose exclusively.
- If clock mode is disabled (`false`) **and** a GUI pose has been received recently (within the last 30 seconds), the robot follows the GUI pose.
- If no recent GUI pose is received within 30 seconds, the controller automatically falls back to the clock pose regardless of the mode flag.
  
---

## Velocity Command Computation

- **Linear Velocity (x)**: Proportional to the distance between the current robot position and the target pose.
- **Angular Velocity (z)**: Proportional to the angular difference between the robot's current orientation and the direction to the target pose.
- Linear velocity is reduced by half if the angular error is greater than 0.5 radians to encourage turning first.
- The controller stops the robot if it is within 5 cm of the target.

---

## `/cmd_vel` Topic Specification

| Detail       | Value                          |
|--------------|--------------------------------|
| **Message Type** | `geometry_msgs/msg/Twist`     |
| **Linear**       | Velocity along x, y, z axes (only x used here) |
| **Angular**      | Angular velocity around x, y, z axes (only z used here) |

---

## Example

The `motion_controller` node continuously receives pose commands and publishes velocity commands to `/cmd_vel`:

```bash
ros2 run motion_controller motion_controller_node
