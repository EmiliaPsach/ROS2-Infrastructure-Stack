#include <gtest/gtest.h>  // Google Test framework

#include <cmath>                       // For std::hypot and std::sqrt
#include <geometry_msgs/msg/pose.hpp>  // ROS 2 pose message type

#include "clock_pose_issuer/clock_pose_issuer.hpp"  // The class under test

using namespace clock_pose_issuer;  // For shorter class references

// Define a test fixture class inheriting from Google Test's Test class
class ClockPoseIssuerTest : public ::testing::Test {
 protected:
  // Called before each test
  void SetUp() override {
    rclcpp::init(0, nullptr);                     // Initialize ROS 2 (with dummy args)
    node_ = std::make_shared<ClockPoseIssuer>();  // Create the node to test
  }

  // Called after each test
  void TearDown() override {
    rclcpp::shutdown();  // Clean up ROS 2
  }

  std::shared_ptr<ClockPoseIssuer> node_;  // Shared pointer to the node under test
};

// Test: Check that the pose returned lies on the unit circle (x² + y² ≈ 1)
TEST_F(ClockPoseIssuerTest, PoseHasUnitLengthPosition) {
  geometry_msgs::msg::Pose pose = node_->calculate_clock_pose();
  double length = std::hypot(pose.position.x, pose.position.y);  // Compute Euclidean distance
  EXPECT_NEAR(length, 1.0, 1e-3);                                // Assert length is approximately 1
}

// Test: Check that the orientation quaternion is normalized (norm ≈ 1)
TEST_F(ClockPoseIssuerTest, OrientationIsNormalized) {
  geometry_msgs::msg::Pose pose = node_->calculate_clock_pose();
  double norm =
      std::sqrt(pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
                pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w);
  EXPECT_NEAR(norm, 1.0, 1e-6);  // Assert quaternion norm is close to 1
}

// Test: Verify time-to-angle conversion keeps angle within expected bounds
TEST_F(ClockPoseIssuerTest, AngleConversionBounds) {
  for (double m = 0; m < 60.0; m += 5.0) {
    double angle = node_->time_to_angle(m);  // Convert minutes to angle
    EXPECT_GE(angle, -M_PI);                 // Check lower bound
    EXPECT_LE(angle, M_PI);                  // Check upper bound
  }
}

// Test: Known time-to-angle mappings (0, 15, 30, 45, 60 minutes)
TEST_F(ClockPoseIssuerTest, TimeToAngleKnownValues) {
  // 0 minutes → π/2 (top of clock)
  EXPECT_NEAR(node_->time_to_angle(0.0), M_PI / 2.0, 1e-6);

  // 15 minutes → 0 (right of clock)
  EXPECT_NEAR(node_->time_to_angle(15.0), 0.0, 1e-6);

  // 30 minutes → -π/2 (bottom of clock)
  EXPECT_NEAR(node_->time_to_angle(30.0), -M_PI / 2.0, 1e-6);

  // 45 minutes → -π (left of clock)
  EXPECT_NEAR(node_->time_to_angle(45.0), -M_PI, 1e-6);

  // 60 minutes → π/2 again (wraps around)
  EXPECT_NEAR(node_->time_to_angle(60.0), M_PI / 2.0, 1e-6);
}
