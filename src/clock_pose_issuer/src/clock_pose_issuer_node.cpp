/* Main entry point to this package that
    - initializes ROS2
    - creates an instance of the class
    - spins the node (keep it running)
    - handles shutdown gracefully
*/

#include "clock_pose_issuer/clock_pose_issuer.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Create the node instance
    auto node = std::make_shared<clock_pose_issuer::ClockPoseIssuer>();
    
    RCLCPP_INFO(node->get_logger(), "Starting clock pose issuer node...");
    
    try {
        // Keep the node running and processing callbacks
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception caught: %s", e.what());
    }
    
    // Clean shutdown
    rclcpp::shutdown();
    return 0;
}