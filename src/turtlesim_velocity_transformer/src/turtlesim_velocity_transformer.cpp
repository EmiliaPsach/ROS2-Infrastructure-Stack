#include "turtlesim_velocity_transformer/turtlesim_velocity_transformer.hpp"

namespace turtlesim_velocity_transformer
{

/**
 * @brief Constructor implementation.
 * 
 * Initializes subscribers for /cmd_vel (world frame velocity commands) and /cur_pose (latest robot pose),
 * and a publisher for turtlesim's /turtle1/cmd_vel (body frame velocity commands).
 */
TurtlesimVelocityTransformer::TurtlesimVelocityTransformer() : Node("turtlesim_velocity_transformer")
{
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&TurtlesimVelocityTransformer::cmdVelCallback, this, std::placeholders::_1));

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/cur_pose", 10,
    std::bind(&TurtlesimVelocityTransformer::poseCallback, this, std::placeholders::_1));

  pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
}

/**
 * @brief Pose callback updates the latest_pose_ and logs current orientation.
 * 
 * Extracts quaternion from the pose message, converts to Euler angles, and logs yaw.
 * 
 * @param msg Incoming pose message with robot position and orientation.
 */
void TurtlesimVelocityTransformer::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  latest_pose_ = msg;

  const auto& q = msg->pose.orientation;
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

  RCLCPP_INFO(this->get_logger(),
    "Pose: x=%.3f, y=%.3f, theta (yaw)=%.3f radians (%.1f degrees)",
    msg->pose.position.x, msg->pose.position.y, yaw, yaw * 180.0 / M_PI);
}

/**
 * @brief Velocity command callback transforms and scales world frame velocities to body frame velocities.
 * 
 * Waits until a latest pose is available. Then, rotates linear velocity vector according to the
 * robot's current yaw angle, scales it to turtlesim's expected speed range, and publishes the
 * transformed command.
 * 
 * @param msg Incoming velocity command in world frame.
 */
void TurtlesimVelocityTransformer::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!latest_pose_) {
    // Warn if no pose has been received yet (throttled to once every 2 seconds)
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for sim_pose...");
    return;
  }

  // Maximum linear speed turtlesim can accept
  const double max_turtlesim_speed = 2.0;

  // Extract linear velocity (x) and angular velocity (z) from the incoming world frame command
  double vx_world = msg->linear.x;   
  double wz_world = msg->angular.z;

  // Extract current yaw from latest pose orientation
  const auto& q = latest_pose_->pose.orientation;
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

  // Rotate linear velocity vector into robot's body frame (only x direction considered)
  // Rotation matrix transpose applied: v_body = R^T * v_world
  double vx_robot = std::cos(-yaw) * vx_world;

  // Scale normalized velocity to turtlesim units
  double vx_robot_scaled = vx_robot * max_turtlesim_speed;

  // Create and populate the velocity message to publish
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = vx_robot_scaled;  // Scaled linear velocity in body frame
  cmd.angular.z = wz_world;        // Angular velocity (no scaling needed)

  // Log details of transformation
  RCLCPP_INFO(this->get_logger(),
    "Cmd_vel (world): linear.x=%.3f, angular.z=%.3f | Pose yaw=%.3f rad | Cmd_vel (body scaled): linear.x=%.3f, angular.z=%.3f",
    vx_world, wz_world, yaw, cmd.linear.x, cmd.angular.z);

  // Publish transformed velocity command
  pub_->publish(cmd);
}

}  // namespace turtlesim_velocity_transformer
