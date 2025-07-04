#include "motion_controller/motion_controller.hpp"

#include <cmath>
#include <functional>
#include "std_msgs/msg/bool.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

using std::placeholders::_1;

namespace motion_controller {

MotionController::MotionController() : Node("motion_controller") {
  // Publisher for velocity commands
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Subscriber for target poses from the clock_pose_issuer
  clock_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose_clock", 10,
      std::bind(&MotionController::clock_pose_callback, this, _1));

  // Subscriber for target poses from the gui_pose_issuer
  gui_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose_gui", 10,
      std::bind(&MotionController::gui_pose_callback, this, _1));

  // Subscriber for current pose of the robot
  current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cur_pose", 10,
      std::bind(&MotionController::current_pose_callback, this, _1));

  // Initialize GUI pose timestamp far in the past to avoid accidental usage
  last_gui_pose_time_ = this->now() - rclcpp::Duration::from_seconds(1000.0);
  has_gui_pose_ = false;

  // Start in clock mode by default
  clock_mode_enabled_ = true;

  // Subscribe to clock mode toggle (true = clock mode enabled, false = GUI mode)
  clock_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/clock_mode_enabled", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      clock_mode_enabled_ = msg->data;
    });
}

// Store the latest clock_pose
void MotionController::clock_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  clock_pose_ = *msg;
}

// Store the latest gui_pose and timestamp it
void MotionController::gui_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  gui_pose_ = *msg;
  last_gui_pose_time_ = this->now();
  has_gui_pose_ = true;
}

// Main control loop: compute velocity commands toward the appropriate target
void MotionController::current_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr current) {
  geometry_msgs::msg::PoseStamped target;

  // Determine whether to follow the GUI pose or fall back to the clock pose
  if (!clock_mode_enabled_ && has_gui_pose_ &&
      (this->now() - last_gui_pose_time_).seconds() < 30.0) {
    target = gui_pose_;
  } else {
    target = clock_pose_;
  }

  // Compute vector from current position to target
  double dx = target.pose.position.x - current->pose.position.x;
  double dy = target.pose.position.y - current->pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  // Stop the robot if close enough to the target
  if (distance < 0.05) {
    cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
    return;
  }

  // Extract current yaw from robot's orientation
  const auto& q = current->pose.orientation;
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

  // Compute desired heading and angle difference
  double desired_yaw = std::atan2(dy, dx);
  double angle_error = std::atan2(std::sin(desired_yaw - yaw), std::cos(desired_yaw - yaw));

  // Velocity gain constants
  const double k_linear = 1.0;
  const double k_angular = 4.0;

  // Proportional linear velocity
  double linear_velocity = k_linear * distance;

  // Reduce speed if facing away from the target
  if (std::abs(angle_error) > 0.5) {
    linear_velocity /= 2;
  }

  // Proportional angular velocity
  double angular_velocity = k_angular * angle_error;

  // Construct and publish Twist message
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear_velocity;
  cmd_vel.angular.z = angular_velocity;

  cmd_vel_publisher_->publish(cmd_vel);
}

}  // namespace motion_controller
