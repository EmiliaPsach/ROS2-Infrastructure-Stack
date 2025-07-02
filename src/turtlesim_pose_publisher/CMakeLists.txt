cmake_minimum_required(VERSION 3.8)
project(turtlesim_pose_publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Include directories
include_directories(include)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add executable
add_executable(turtlesim_pose_publisher src/turtlesim_pose_publisher.cpp)

# Link dependencies
ament_target_dependencies(turtlesim_pose_publisher
  rclcpp
  turtlesim
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# Install the executable
install(TARGETS turtlesim_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Export package info
ament_package()
