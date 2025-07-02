cmake_minimum_required(VERSION 3.8...3.27)
project(turtlesim_velocity_transformer)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Create a library from the class implementation
add_library(${PROJECT_NAME}_lib 
  src/turtlesim_velocity_transformer.cpp
)

# Link dependencies to library
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  geometry_msgs
)

# Create executable
add_executable(turtlesim_velocity_transformer 
  src/turtlesim_velocity_transformer_node.cpp
)

# Link library to executable
target_link_libraries(turtlesim_velocity_transformer ${PROJECT_NAME}_lib)

# Link dependencies to executable
ament_target_dependencies(turtlesim_velocity_transformer
  rclcpp
  geometry_msgs
)

# Install executable
install(TARGETS
  turtlesim_velocity_transformer
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers (for other packages to use)
install(DIRECTORY include/
  DESTINATION include/
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(rclcpp geometry_msgs)

ament_package()