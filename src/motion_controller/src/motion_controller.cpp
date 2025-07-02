/* Implements the class interface's methods for this package */

#include "motion_controller/motion_controller.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <cmath>
#include <functional>

using std::placeholders::_1;
using std::placeholders::_2;

namespace motion_controller
{

MotionController::MotionController()
: Node("motion_controller")
{
    // Publisher for velocity commands
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Subscribers using message_filters for time synchronization
    target_pose_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(this, "/target_pose_clock");
    current_pose_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(this, "/cur_pose");

    // Synchronizer to call control logic only when both messages are available
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *target_pose_sub_, *current_pose_sub_);
    sync_->registerCallback(std::bind(&MotionController::synced_callback, this, _1, _2));
}

// Callback for synchronized target and current pose messages
void MotionController::synced_callback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr &target,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr &current)
{
    // Compute position difference in world frame
    double dx = target->pose.position.x - current->pose.position.x;
    double dy = target->pose.position.y - current->pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    RCLCPP_INFO(this->get_logger(), "dx: %.2f, dy: %.2f, distance: %.2f", dx, dy, distance);

    if (distance < 0.05)
    {
        RCLCPP_INFO(this->get_logger(), "Target reached. Stopping.");
        cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
        return;
    }

    // Extract current yaw (heading) from current pose quaternion
    const auto &q = current->pose.orientation;
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

    // Compute desired heading angle to target
    double desired_yaw = std::atan2(dy, dx);

    // Compute shortest angular difference between current heading and desired heading
    double angle_error = std::atan2(std::sin(desired_yaw - yaw), std::cos(desired_yaw - yaw));

    // Proportional controller gains (tune as needed)
    const double k_linear = 1.0;
    const double k_angular = 4.0;

    // Linear velocity proportional to distance but slow down when angle error is large
    double linear_velocity = k_linear * distance;
    if (std::abs(angle_error) > 0.5)  // if heading error too big, reduce forward speed
    {
        linear_velocity /= 2;
    }

    // Angular velocity proportional to angle error
    double angular_velocity = k_angular * angle_error;

    // Publish Twist with linear.x and angular.z
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear_velocity;
    cmd_vel.angular.z = angular_velocity;

    RCLCPP_INFO(this->get_logger(), "linear.x=%.3f, angular.z=%.3f", linear_velocity, angular_velocity);

    cmd_vel_publisher_->publish(cmd_vel);
}



}  // namespace motion_controller
