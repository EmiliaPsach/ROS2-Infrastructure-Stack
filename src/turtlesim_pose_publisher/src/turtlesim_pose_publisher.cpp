#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class TurtlesimPosePublisher : public rclcpp::Node
{
public:
  TurtlesimPosePublisher() : Node("turtlesim_pose_publisher")
  {
    pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10,
      std::bind(&TurtlesimPosePublisher::pose_callback, this, std::placeholders::_1));

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/sim_pose", 10);
  }

private:
  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = this->get_clock()->now();
      pose_msg.header.frame_id = "map";

      const double width = 11.0889;
      const double height = 11.0889;

      // Center coordinates
      double x_centered = msg->x - width / 2.0;
      double y_centered = msg->y - height / 2.0;

      // Scale to world frame using circular region inside full [-1, 1] x [-1, 1] box
      pose_msg.pose.position.x = (x_centered / (width / 2.0));
      pose_msg.pose.position.y = (y_centered / (height / 2.0));
      pose_msg.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, msg->theta);
      pose_msg.pose.orientation = tf2::toMsg(q);

      pose_pub_->publish(pose_msg);
  }


  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlesimPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
