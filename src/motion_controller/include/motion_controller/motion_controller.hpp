/* Declares the class interface for this package */

#ifndef MOTION_CONTROLLER_HPP
#define MOTION_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

namespace motion_controller
{

class MotionController : public rclcpp::Node
{
public:
    MotionController();

private:
    // Callback for synchronized target and current pose messages
    void synced_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &target,
                         const geometry_msgs::msg::PoseStamped::ConstSharedPtr &current);

    // Publisher for cmd_vel velocity commands
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    // Message filter subscribers for target and current poses
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> target_pose_sub_;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> current_pose_sub_;

    // Synchronizer for approximate time policy
    typedef message_filters::sync_policies::ApproximateTime<
        geometry_msgs::msg::PoseStamped,
        geometry_msgs::msg::PoseStamped> SyncPolicy;

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

}  // namespace motion_controller

#endif  // MOTION_CONTROLLER_HPP
