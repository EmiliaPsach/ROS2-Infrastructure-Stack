#include "motion_controller/motion_controller.hpp"

#include <cmath>
#include <functional>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

using std::placeholders::_1;

namespace motion_controller {

MotionController::MotionController() : Node("motion_controller") {
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Subscribe to clock target pose
  clock_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose_clock", 10,
      std::bind(&MotionController::clock_pose_callback, this, _1));

  // Subscribe to gui target pose
  gui_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose_gui", 10,
      std::bind(&MotionController::gui_pose_callback, this, _1));

  // Subscribe to current pose
  current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cur_pose", 10,
      std::bind(&MotionController::current_pose_callback, this, _1));

  // Initialize last GUI command time as zero time
  last_gui_pose_time_ = this->now() - rclcpp::Duration::from_seconds(1000.0);
  has_gui_pose_ = false;
}

void MotionController::clock_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  clock_pose_ = *msg;
}

void MotionController::gui_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  gui_pose_ = *msg;
  last_gui_pose_time_ = this->now();
  has_gui_pose_ = true;
}

void MotionController::current_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr current) {
  geometry_msgs::msg::PoseStamped target;

  // Check if GUI pose is recent enough (less than 30 seconds ago)
  if (has_gui_pose_ && (this->now() - last_gui_pose_time_).seconds() < 30.0) {
    target = gui_pose_;
  } else {
    target = clock_pose_;
  }

  // Compute dx, dy
  double dx = target.pose.position.x - current->pose.position.x;
  double dy = target.pose.position.y - current->pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  if (distance < 0.05) {
    cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
    return;
  }

  const auto& q = current->pose.orientation;
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

  double desired_yaw = std::atan2(dy, dx);
  double angle_error = std::atan2(std::sin(desired_yaw - yaw), std::cos(desired_yaw - yaw));

  const double k_linear = 1.0;
  const double k_angular = 4.0;

  double linear_velocity = k_linear * distance;
  if (std::abs(angle_error) > 0.5) {
    linear_velocity /= 2;
  }

  double angular_velocity = k_angular * angle_error;

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear_velocity;
  cmd_vel.angular.z = angular_velocity;

  cmd_vel_publisher_->publish(cmd_vel);
}

}  // namespace motion_controller
