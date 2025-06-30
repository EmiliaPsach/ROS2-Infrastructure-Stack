/* Implements the class interface's methods for this package */

#include "clock_pose_issuer/clock_pose_issuer.hpp"
#include <cmath>

using namespace std::chrono_literals;

constexpr const char* clock_pose_issuer::ClockPoseIssuer::DEFAULT_TOPIC_NAME;

namespace clock_pose_issuer
{

ClockPoseIssuer::ClockPoseIssuer() : Node("clock_pose_issuer")
{
    // Declare and get parameters
    this->declare_parameter("publish_rate", 10.0);
    this->declare_parameter("frame_id", "map");
    this->declare_parameter("topic_name", DEFAULT_TOPIC_NAME);
    
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    frame_id_ = this->get_parameter("frame_id").as_string();
    topic_name_ = this->get_parameter("topic_name").as_string();
    
    // Validate parameters
    if (publish_rate_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "Invalid publish_rate: %.2f, using default 10.0", publish_rate_);
        publish_rate_ = 10.0;
    }
    
    // Create publisher
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        topic_name_, 10);
    
    // Create timer with calculated period
    auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = this->create_wall_timer(
        timer_period, std::bind(&ClockPoseIssuer::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), 
        "Clock pose issuer initialized:");
    RCLCPP_INFO(this->get_logger(), "  - Publishing to: %s", topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Rate: %.1f Hz", publish_rate_);
    RCLCPP_INFO(this->get_logger(), "  - Frame ID: %s", frame_id_.c_str());
}

void ClockPoseIssuer::timer_callback()
{
    auto message = geometry_msgs::msg::PoseStamped();
    
    // Set header
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = frame_id_;
    
    // Calculate and set pose
    message.pose = calculate_clock_pose();
    
    // Publish
    pose_publisher_->publish(message);
    
    RCLCPP_DEBUG(this->get_logger(), 
        "Published pose: (%.3f, %.3f)", 
        message.pose.position.x, 
        message.pose.position.y);
}

geometry_msgs::msg::Pose ClockPoseIssuer::calculate_clock_pose() const
{
    // Get current time
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t);
    
    // Get milliseconds for smooth movement
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    // Calculate precise minutes (includes seconds and milliseconds)
    double minutes = tm.tm_min + 
                    tm.tm_sec / SECONDS_PER_MINUTE + 
                    ms.count() / (SECONDS_PER_MINUTE * 1000.0);
    
    // Convert to angle and then to pose
    double angle = time_to_angle(minutes);
    
    geometry_msgs::msg::Pose pose;
    
    // Position on unit circle
    pose.position.x = cos(angle);
    pose.position.y = sin(angle);
    pose.position.z = 0.0;
    
    // Orientation: robot faces toward center of clock
    // (opposite direction of position vector)
    double facing_angle = angle + M_PI;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = sin(facing_angle / 2.0);
    pose.orientation.w = cos(facing_angle / 2.0);
    
    return pose;
}

double ClockPoseIssuer::time_to_angle(double minutes) const
{
    // Clamp to [0, 60) to wrap around if needed
    double minute = fmod(minutes, 60.0);

    // Convert minutes to angle on clock face
    // 0 minutes = top = π/2 radians
    // 15 minutes = right = 0 radians
    // 30 minutes = bottom = -π/2 radians
    // 45 minutes = left = π radians
    
    double angle = (M_PI / 2.0) - (minute * 2.0 * M_PI / 60.0);
    
    return angle;
}

} // namespace clock_pose_issuer