# Motion Controller
The `motion_controller` ROS 2 package is responsible for controlling the robot's movement based on pose commands received from two different sources: the `clock_pose_issuer` and the `gui_pose_issuer`. This node reads target poses from these publishers and computes the appropriate velocity commands to move the robot to the desired location.

## `/cmd_vel` Topic

### Specification

| Detail | Value |
|---|---|
| **Message Type** | `geometry_msgs/msg/Twist` |
| **Linear** | Defines the robot’s linear velocity along the x, y, and z axes. |
| **Angular** | Defines the robot’s angular velocity around the z, y, and x axes. |

### Target Pose Selection Logic

The robot can either follow the position dictated by the `clock_pose_issuer` (which corresponds to a position on a unit circle based on the time of day) or follow a pose commanded by a human driver through the `gui_pose_issuer`. If no new pose command is received from the `gui_pose_issuer` for 30 seconds, the robot will automatically return to following the `clock_pose_issuer`.

### Velocity Semantics

#### Linear velocity (x, y)

The robot's movement is directed toward the target pose, which is calculated based on the incoming pose command.

#### Angular velocity (z)

The robot is oriented to face the target pose by rotating toward it. The angular velocity is adjusted accordingly to move toward the desired orientation.