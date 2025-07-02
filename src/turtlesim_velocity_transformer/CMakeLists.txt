cmake_minimum_required(VERSION 3.5)
project(turtlesim_velocity_transformer)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(turtlesim_velocity_transformer src/turtlesim_velocity_transformer.cpp)

ament_target_dependencies(turtlesim_velocity_transformer
  rclcpp
  geometry_msgs
)

install(TARGETS turtlesim_velocity_transformer
  DESTINATION lib/${PROJECT_NAME})

ament_package()
