#ifndef TURTLESIM_POSE_TRANSFORMER_HPP_
#define TURTLESIM_POSE_TRANSFORMER_HPP_

// ROS 2 core and message includes
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace turtlesim_pose_transformer
{
/**
 * @brief A ROS 2 node that subscribes to the turtlesim pose topic and publishes
 *        a transformed PoseStamped message in a normalized world frame.
 */
class TurtlesimPosePublisher : public rclcpp::Node
{
public:
  /**
   * @brief Constructor. Initializes subscriptions and publishers.
   */
  TurtlesimPosePublisher();

private:
  /**
   * @brief Callback function called when a new turtlesim pose message is received.
   *        It converts the turtlesim pose to a normalized PoseStamped message and publishes it.
   * 
   * @param msg Shared pointer to the incoming turtlesim::msg::Pose message.
   */
  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg);

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;  ///< Subscription to turtlesim pose topic
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;  ///< Publisher for normalized PoseStamped messages
};

#endif  // TURTLESIM_POSE_TRANSFORMER_HPP_

} // namespace turtlesim_pose_transformer