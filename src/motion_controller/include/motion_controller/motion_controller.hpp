#ifndef MOTION_CONTROLLER__MOTION_CONTROLLER_HPP_
#define MOTION_CONTROLLER__MOTION_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

namespace motion_controller {

class MotionController : public rclcpp::Node {
 public:
  MotionController();

 private:
  // Pose subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr clock_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gui_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;

  // Mode subscriber
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clock_mode_sub_;

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  // State
  geometry_msgs::msg::PoseStamped clock_pose_;
  geometry_msgs::msg::PoseStamped gui_pose_;
  rclcpp::Time last_gui_pose_time_;
  bool has_gui_pose_;
  bool clock_mode_enabled_;

  // Callbacks
  void clock_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void gui_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void current_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};

}  // namespace motion_controller

#endif  // MOTION_CONTROLLER__MOTION_CONTROLLER_HPP_
