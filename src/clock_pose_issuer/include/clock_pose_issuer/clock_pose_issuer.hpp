/* Declares the class interface for this package */

#ifndef CLOCK_POSE_ISSUER_HPP_
#define CLOCK_POSE_ISSUER_HPP_

#include <chrono>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace clock_pose_issuer {

class ClockPoseIssuer : public rclcpp::Node {
 public:
  /**
   * @brief Constructor - initializes the node
   */
  ClockPoseIssuer();

  /**
   * @brief Destructor
   */
  ~ClockPoseIssuer() = default;

  /**
   * @brief Calculate pose based on current time
   * @return Pose representing current time on clock face
   */
  geometry_msgs::msg::Pose calculate_clock_pose() const;

  /**
   * @brief Convert time components to angle on unit circle
   * @param minutes Current minutes (including fractional seconds)
   * @return Angle in radians
   */
  double time_to_angle(double minutes) const;

 private:
  /**
   * @brief Timer callback that publishes clock poses
   */
  void timer_callback();

  // Topic name
  static constexpr const char* DEFAULT_TOPIC_NAME = "/target_pose_clock";

  // ROS2 components
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

  // Configuration parameters
  double publish_rate_;
  std::string frame_id_;
  std::string topic_name_;

  // Constants
  static constexpr double SECONDS_PER_MINUTE = 60.0;
  static constexpr double MINUTES_PER_HOUR = 60.0;
};

}  // namespace clock_pose_issuer

#endif  // CLOCK_POSE_ISSUER_HPP_