/* Implements the class interface's methods for this package */

#include "motion_controller/motion_controller.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"  // For current pose
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace motion_controller
{

    MotionController::MotionController() : Node("motion_controller"), is_clock_following_(true)
{
    // Declare parameters
    this->declare_parameter("publish_rate", 10.0);
    this->declare_parameter("frame_id", "map");

    // Create publisher for cmd_vel (the robot's velocity command topic)
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Create subscriber for the target pose (PoseStamped) from the clock_pose_issuer or any other source
    target_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/target_pose_clock", 10,
        std::bind(&MotionController::target_pose_callback, this, std::placeholders::_1)
    );

    // Create subscriber for the current pose (odometry) of the robot
    current_pose_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&MotionController::current_pose_callback, this, std::placeholders::_1)
    );

    // Add a timer to periodically call move_towards_target()
    auto timer_period = std::chrono::milliseconds(100);  // 10Hz
    move_timer_ = this->create_wall_timer(
        timer_period, std::bind(&MotionController::move_towards_target, this)
    );

    // Add a timer to periodically check if we need to return to clock-following mode
    auto return_timer_period = std::chrono::seconds(30);
    this->create_wall_timer(return_timer_period, std::bind(&MotionController::return_to_clock_following, this));
}


void MotionController::target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // Access the pose inside the PoseStamped message
    target_pose_ = *msg;  // Now target_pose_ is of type PoseStamped
    // RCLCPP_INFO(this->get_logger(), "Received target pose: x: %f, y: %f", target_pose_.pose.position.x, target_pose_.pose.position.y);
}

void MotionController::current_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Access the current pose of the robot from the odometry message
    current_pose_.pose = msg->pose.pose;
    RCLCPP_INFO(this->get_logger(), "Received current pose: x: %f, y: %f", current_pose_.pose.position.x, current_pose_.pose.position.y);
}

void MotionController::return_to_clock_following()
{
    // This method should be called when the robot should return to following clock poses.
    // If the robot is not moving towards a target, stop it and return to clock-following mode.

    if (is_clock_following_)
    {
        RCLCPP_INFO(this->get_logger(), "Returning to clock-following mode...");

        // Implement logic to stop movement or return to clock-following mode
        geometry_msgs::msg::Twist stop_msg;
        cmd_vel_publisher_->publish(stop_msg);  // Stop the robot (send zero velocity)
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Not in clock-following mode, cannot return.");
    }
}

void MotionController::move_towards_target()
{
    // Calculate the error between current and target poses
    double x_error = target_pose_.pose.position.x - current_pose_.pose.position.x;
    double y_error = target_pose_.pose.position.y - current_pose_.pose.position.y;
    double distance_error = std::sqrt(x_error * x_error + y_error * y_error);

    geometry_msgs::msg::Twist msg;

    // If we're close enough to the target, stop the robot
    if (distance_error < 0.1) {
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        cmd_vel_publisher_->publish(msg);
        // RCLCPP_INFO(this->get_logger(), "Reached target pose, stopping...");
        return;
    }

    // Control linear velocity (move towards the target)
    msg.linear.x = 0.5;  // Move at a constant speed

    // Control angular velocity (turn towards the target)
    double angle_error = atan2(y_error, x_error);
    msg.angular.z = 0.5 * angle_error;  // 0.5 is a gain factor

    // Log the calculated velocity to verify
    RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear.x = %.2f, angular.z = %.2f", msg.linear.x, msg.angular.z);

    // Publish the velocity command
    cmd_vel_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Moving towards target pose: x: %.2f, y: %.2f", target_pose_.pose.position.x, target_pose_.pose.position.y);
}

}  // namespace motion_controller
