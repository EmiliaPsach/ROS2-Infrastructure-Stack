#include "gui_pose_issuer/gui_pose_issuer.hpp"
#include "rclcpp/rclcpp.hpp"
#include <QApplication>
#include <thread>

int main(int argc, char* argv[]) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Initialize Qt application
  QApplication app(argc, argv);

  // Create the GUI node
  auto node = std::make_shared<gui_pose_issuer::GuiPoseIssuer>();

  RCLCPP_INFO(node->get_logger(), "Starting GUI pose issuer node...");

  // Spin ROS 2 in a separate thread so it doesn't block Qt event loop
  std::thread ros_spin_thread([&]() {
    rclcpp::spin(node);
  });

  // Run the Qt event loop (this blocks until GUI is closed)
  int ret = app.exec();

  // When GUI closes, shutdown ROS
  rclcpp::shutdown();

  // Wait for ROS spin thread to finish
  ros_spin_thread.join();

  return ret;
}
