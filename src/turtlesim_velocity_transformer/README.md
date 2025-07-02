# Turtlesim Velocity Transformer

The `turtlesim_velocity_transformer` node converts velocity commands from a normalized world frame (`[-1, 1] × [-1, 1]`) into `turtlesim`-compatible body frame commands. The transformer waits until it receives an initial pose before performing any velocity conversion.

It listens to the robot's current pose, performs frame rotation, applies scaling to match `turtlesim`'s velocity range, and publishes the transformed command.

## Output Topics

### `/turtle1/cmd_vel`

| Detail | Value |
|---|---|
| **Message Type** | `geometry_msgs/msg/Twist` |
| **Linear Velocity** | **x**: In the turtle’s body frame (scaled from world frame) |
| **Angular Velocity** | **z**: Forwarded directly from world frame command |

## Input Topics

### `/cmd_vel`

| Detail | Value |
|---|---|
| **Message Type** | `geometry_msgs/msg/Twist` |
| **Semantics** | Desired velocity in **world frame** (linear.x only, angular.z optional) |
| **Units** | Normalized to `[-1, 1]` range |

### `/cur_pose`

| Detail | Value |
|---|---|
| **Message Type** | `geometry_msgs/msg/PoseStamped` |
| **Header.frame_id** | Arbitrary, but must match source of pose estimation |
| **Semantics** | Latest robot pose used to rotate velocity vectors into body frame |

## Velocity Transformation Logic

### Scaling and Rotation

1. **Rotation**: The incoming linear velocity in the x-direction (world frame) is rotated into the robot's body frame using the inverse of the current yaw.
2. **Scaling**: The rotated velocity is scaled to match `turtlesim`’s velocity range (default max speed: `2.0` units/sec).
3. **Angular Velocity**: The `angular.z` component is passed through without scaling, assuming it's already in `turtlesim` units.

| Input (`/cmd_vel`) | Pose (`/cur_pose`) | Output (`/turtle1/cmd_vel`) |
|---|---|---|
| linear.x (world frame) | orientation (quaternion) → yaw | linear.x (body frame, scaled) |
| angular.z (world frame) | — | angular.z (passed through) |

### Example

If the robot is facing 90° counter-clockwise (i.e., yaw = π/2 rad), and receives a forward command in the world frame:

- **World command**: `{linear.x = 1.0}`
- **Rotation into body frame**: `cos(-π/2) * 1.0 = 0.0`
- **Output**: No forward movement, as the desired world direction is now perpendicular to the robot's heading.
