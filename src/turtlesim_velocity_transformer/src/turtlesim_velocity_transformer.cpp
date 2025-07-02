#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <cmath>

/**
 * @brief Node to transform velocity commands from world frame to the turtle's body frame.
 * 
 * It reads velocity commands published in world coordinates (from motion_controller),
 * rotates them using the latest robot orientation from /sim_pose, scales velocity
 * to turtlesim units, and publishes them to turtlesim's /turtle1/cmd_vel topic (body frame).
 */
class TurtlesimVelocityTransformer : public rclcpp::Node
{
public:
  TurtlesimVelocityTransformer() : Node("turtlesim_velocity_transformer")
  {
    // Subscribe to normalized velocity commands in world frame (from motion_controller)
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&TurtlesimVelocityTransformer::cmdVelCallback, this, std::placeholders::_1));

    // Subscribe to the current robot pose to get orientation
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/sim_pose", 10,
      std::bind(&TurtlesimVelocityTransformer::poseCallback, this, std::placeholders::_1));

    // Publisher to the turtlesim velocity command topic (body frame)
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
  }

private:
  geometry_msgs::msg::PoseStamped::SharedPtr latest_pose_;

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    latest_pose_ = msg;

    // Extract quaternion from pose
    const auto& q = msg->pose.orientation;
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

    // Convert quaternion to Euler angles (roll, pitch, yaw)
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(this->get_logger(),
      "Pose: x=%.3f, y=%.3f, theta (yaw)=%.3f radians (%.1f degrees)",
      msg->pose.position.x, msg->pose.position.y, yaw, yaw * 180.0 / M_PI);
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!latest_pose_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for sim_pose...");
      return;
    }

    // Parameters
    const double max_turtlesim_speed = 2.0;  // tune as needed for turtlesim max linear speed

    // Extract commanded velocities from motion_controller (world frame, normalized [-1,1])
    double vx_world = msg->linear.x;     // forward velocity in world frame (normalized)
    double wz_world = msg->angular.z;    // angular velocity around z axis (rad/s)

    // Extract current orientation quaternion from latest_pose_
    const auto& q = latest_pose_->pose.orientation;
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

    // Rotate linear velocity vector (vx_world, 0) into robot's body frame
    // Body frame velocity = R^T * world velocity, R is rotation by yaw
    double vx_robot = std::cos(-yaw) * vx_world;

    // Scale linear velocity from normalized to turtlesim units
    double vx_robot_scaled = vx_robot * max_turtlesim_speed;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = vx_robot_scaled;  // forward velocity in body frame (m/s)
    cmd.angular.z = wz_world;        // angular velocity in rad/s (use directly)

    RCLCPP_INFO(this->get_logger(),
      "Cmd_vel (world): linear.x=%.3f, angular.z=%.3f | Pose yaw=%.3f rad | Cmd_vel (body scaled): linear.x=%.3f, angular.z=%.3f",
      vx_world, wz_world, yaw, cmd.linear.x, cmd.angular.z);

    pub_->publish(cmd);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlesimVelocityTransformer>());
  rclcpp::shutdown();
  return 0;
}
