#ifndef TURTLESIM_VELOCITY_TRANSFORMER_HPP_
#define TURTLESIM_VELOCITY_TRANSFORMER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <cmath>

namespace turtlesim_velocity_transformer
{

/**
 * @brief Node to transform velocity commands from world frame to the turtle's body frame.
 * 
 * It reads velocity commands published in world coordinates (from motion_controller),
 * rotates them using the latest robot orientation from /cur_pose, scales velocity
 * to turtlesim units, and publishes them to turtlesim's /turtle1/cmd_vel topic (body frame).
 */
class TurtlesimVelocityTransformer : public rclcpp::Node
{
public:
  /**
   * @brief Constructor sets up subscriptions and publisher.
   */
  TurtlesimVelocityTransformer();

private:
  /**
   * @brief Callback for receiving the latest robot pose.
   * 
   * Stores the latest pose and extracts orientation for use in velocity transformation.
   * 
   * @param msg The incoming PoseStamped message containing robot pose and orientation.
   */
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * @brief Callback for velocity commands in the world frame.
   * 
   * Transforms the linear velocity into the turtle's body frame using the latest orientation,
   * scales it appropriately, and publishes it to turtlesim.
   * 
   * @param msg The incoming Twist message containing velocity commands.
   */
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;          ///< Subscriber to normalized velocity commands in world frame
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;      ///< Subscriber to the robot's latest pose
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;                    ///< Publisher to turtlesim velocity commands (body frame)

  geometry_msgs::msg::PoseStamped::SharedPtr latest_pose_;  ///< Storage for latest pose to use in velocity transformation
};

#endif  // TURTLESIM_VELOCITY_TRANSFORMER_HPP_

}