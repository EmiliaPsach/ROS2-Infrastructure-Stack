#include "gui_pose_issuer/gui_pose_issuer.hpp"
#include "rclcpp/rclcpp.hpp"
#include <QApplication>
#include <QTimer>

int main(int argc, char* argv[]) {
  // Initialize Qt
  QApplication app(argc, argv);

  // Initialize ROS2 (after Qt, to avoid conflicts)
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gui_pose_issuer::GuiPoseIssuer>();

  RCLCPP_INFO(node->get_logger(), "Starting GUI pose issuer node...");

  // Use a QTimer to spin ROS2 periodically without blocking the GUI
  QTimer ros_timer;
  QObject::connect(&ros_timer, &QTimer::timeout, [node]() {
    rclcpp::spin_some(node);
  });
  ros_timer.start(10);  // spin ROS every 10 ms

  // Run Qt main event loop (this blocks, but ROS spins through QTimer)
  int result = app.exec();

  // Clean shutdown
  rclcpp::shutdown();
  return result;
}
