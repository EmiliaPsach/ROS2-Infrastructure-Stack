# Clock Pose Issuer

The clock_pose_issuer node publishes 6-DoF target poses to the `/target_pose_clock` topic at a configurable rate (default: 10 Hz). These poses correspond to the position and orientation a robot should move to based on the current minutes hand of a clock.

## `/target_pose_clock` Topic

### Specification

| Detail | Value |
|---|---|
| **Message Type** | `geometry_msgs/msg/PoseStamped` |
| **Header** | **frame_id**: Configurable via the `frame_id` parameter (default: `"map"`). **stamp**: Current ROS time, respecting simulation time if `use_sim_time` is set. |

### Pose Semantics

#### Position (x, y)

The robot’s target location lies on the unit circle, where the angle is derived from the current time. For example,

| Time | Coordinates | Clock Position |
|---|---|---|
| 0 minutes | (0, 1) | 5:00pm (top) |
| 15 minutes | (1, 0) | 5:15pm (right) |
| 30 minutes | (0, -1) | 5:30pm (bottom) |
| 45 minutes | (-1, 0) | 5:45pm (left) |


#### Orientation (x, y, z, w)

The robot is oriented to face toward the center of the clock. This is calculated by rotating the position vector 180° (i.e., facing the origin from its location on the circle). The orientation is expressed as a quaternion in the 2D plane (no pitch or roll).