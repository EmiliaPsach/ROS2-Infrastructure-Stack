#ifndef MOTION_CONTROLLER_HPP_
#define MOTION_CONTROLLER_HPP_

#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace motion_controller {

class MotionController : public rclcpp::Node {
 public:
  MotionController();

 private:
  // Subscribers for target poses and current pose
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr clock_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gui_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;

  // Publisher for velocity commands
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  // Last received poses
  geometry_msgs::msg::PoseStamped clock_pose_;
  geometry_msgs::msg::PoseStamped gui_pose_;

  // Timestamp of last GUI pose received
  rclcpp::Time last_gui_pose_time_;

  // Flag indicating whether a GUI pose has been received
  bool has_gui_pose_;

  // Callbacks
  void clock_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void gui_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void current_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};

}  // namespace motion_controller

#endif  // MOTION_CONTROLLER_HPP_
