# Turtlesim Pose Transformer

The `turtlesim_pose_transformer` node transforms native `/turtle1/pose` messages from `turtlesim` into normalized `geometry_msgs/msg/PoseStamped` messages within a fixed coordinate frame `[-1, 1] × [-1, 1]`. These transformed poses are published on the `/cur_pose` topic for use in other systems that expect frame-consistent input.

## Output Topic

### `/cur_pose`

| Detail | Value |
|---|---|
| **Message Type** | `geometry_msgs/msg/PoseStamped` |
| **Header.frame_id** | `"map"` (fixed) |
| **Header.stamp** | Current ROS time |

#### Pose Semantics

##### Position (x, y)

- The turtle's position is first **centered** around the midpoint of the `turtlesim` window (default dimensions: 11.0889 × 11.0889).
- The result is then **normalized** to fall within `[-1, 1]` in both x and y.

| Turtlesim Position | Normalized Position |
|---|---|
| (5.544, 5.544) | (0.0, 0.0) |
| (0.0, 5.544) | (-1.0, 0.0) |
| (11.0889, 5.544) | (1.0, 0.0) |
| (5.544, 0.0) | (0.0, -1.0) |
| (5.544, 11.0889) | (0.0, 1.0) |

##### Orientation (x, y, z, w)

- The `theta` value (yaw angle in radians) from `turtlesim::msg::Pose` is converted to a quaternion.
- The orientation is strictly 2D (no pitch or roll), and rotation is expressed around the z-axis.

## Input Topic

### `/turtle1/pose`

| Detail | Value |
|---|---|
| **Message Type** | `turtlesim/msg/Pose` |
| **Semantics** | Native 2D pose from `turtlesim`, includes x, y position and heading angle `theta` |
| **Units** | Position in `turtlesim` window coordinates; `theta` in radians |

## Notes

- The normalized pose allows seamless integration with controllers or planners operating in abstract world coordinate systems (e.g., `[-1, 1]`).
- The transformation assumes a fixed `turtlesim` window size of `11.0889 × 11.0889`.

## Example

If the turtle is located at (2.7722, 8.3167) in `turtlesim`, this will be transformed as follows:

- Centered: `(x - 5.5444, y - 5.5444) = (-2.7722, 2.7722)`
- Normalized: `(-0.5, 0.5)`
- Orientation: Converted to quaternion based on current `theta`

This results in a pose in the upper-left quadrant of the normalized world frame.

