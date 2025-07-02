#include "turtlesim_pose_publisher/turtlesim_pose_publisher.hpp"

namespace turtlesim_pose_publisher
{
/**
 * @brief Constructor implementation
 * 
 * Sets up the subscription to "/turtle1/pose" and publisher to "/cur_pose".
 */
TurtlesimPosePublisher::TurtlesimPosePublisher() : Node("turtlesim_pose_publisher")
{
  // Subscribe to turtlesim pose updates with queue size 10
  pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
    "/turtle1/pose", 10,
    std::bind(&TurtlesimPosePublisher::pose_callback, this, std::placeholders::_1));

  // Publisher for transformed pose in PoseStamped format
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/cur_pose", 10);
}

/**
 * @brief Callback to process turtlesim pose messages
 * 
 * Converts the turtlesim pose into a normalized pose centered at (0,0) in a
 * fixed coordinate frame [-1,1] x [-1,1], then publishes the result.
 * 
 * @param msg The turtlesim pose message received
 */
void TurtlesimPosePublisher::pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
{
  geometry_msgs::msg::PoseStamped pose_msg;

  // Timestamp the new message with current ROS time
  pose_msg.header.stamp = this->get_clock()->now();

  // Set the fixed frame id for this pose
  pose_msg.header.frame_id = "map";

  // Turtlesim window dimensions (assumed fixed)
  const double width = 11.0889;
  const double height = 11.0889;

  // Center the pose around the middle of the turtlesim window
  double x_centered = msg->x - width / 2.0;
  double y_centered = msg->y - height / 2.0;

  // Normalize coordinates to range [-1, 1]
  pose_msg.pose.position.x = (x_centered / (width / 2.0));
  pose_msg.pose.position.y = (y_centered / (height / 2.0));
  pose_msg.pose.position.z = 0.0;

  // Convert turtlesim theta (yaw) to quaternion orientation
  tf2::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  pose_msg.pose.orientation = tf2::toMsg(q);

  // Publish the normalized pose
  pose_pub_->publish(pose_msg);
}

/**
 * @brief Main function to initialize ROS, run the node, and clean up.
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create node and spin until shutdown
  rclcpp::spin(std::make_shared<TurtlesimPosePublisher>());

  rclcpp::shutdown();
  return 0;
}

} // namespace turtlesim_pose_publisher