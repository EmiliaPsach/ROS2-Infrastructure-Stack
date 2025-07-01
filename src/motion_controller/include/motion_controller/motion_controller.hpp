/* Declares the class interface for this package */

#ifndef MOTION_CONTROLLER_HPP
#define MOTION_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"  // For current pose (odometry)

namespace motion_controller
{

class MotionController : public rclcpp::Node
{
public:
    MotionController();  // Constructor declaration

private:
    // Callback function to handle incoming target pose messages
    void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // Callback function to handle incoming current pose (odometry) messages
    void current_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Method to handle the transition back to clock-following mode
    void return_to_clock_following();

    // Method to control movement towards the target pose
    void move_towards_target();

    // Timer to periodically call move_towards_target()
    rclcpp::TimerBase::SharedPtr move_timer_;

    // The target pose the robot is trying to reach (of type PoseStamped, which includes a timestamp and frame_id)
    geometry_msgs::msg::PoseStamped target_pose_;  

    // The current pose of the robot (used for comparison with the target pose)
    geometry_msgs::msg::PoseStamped current_pose_;  

    // Publisher for the velocity command (cmd_vel)
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    // Subscriber for the target pose (PoseStamped messages)
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscriber_;

    // Subscriber for the current pose (odometry messages)
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_pose_subscriber_;

    // Flag indicating whether the robot is following the clock (set to true initially)
    bool is_clock_following_;
};

}  // namespace motion_controller

#endif  // MOTION_CONTROLLER_HPP
