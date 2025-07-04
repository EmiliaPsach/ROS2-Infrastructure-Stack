cmake_minimum_required(VERSION 3.8...3.27)
project(clock_pose_issuer)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Create a library from the class implementation
add_library(${PROJECT_NAME}_lib 
  src/clock_pose_issuer.cpp
)

# Link dependencies to library
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  geometry_msgs
)

# Create executable
add_executable(clock_node 
  src/clock_pose_issuer_node.cpp
)

# Link library to executable
target_link_libraries(clock_node ${PROJECT_NAME}_lib)

# Link dependencies to executable
ament_target_dependencies(clock_node
  rclcpp
  std_msgs
  geometry_msgs
)

# Install executable
install(TARGETS
  clock_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers (for other packages to use)
install(DIRECTORY include/
  DESTINATION include/
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(rclcpp std_msgs geometry_msgs)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_clock_pose_issuer test/test_clock_pose_issuer.cpp)
  if(TARGET test_clock_pose_issuer)
    target_link_libraries(test_clock_pose_issuer ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_clock_pose_issuer
      rclcpp
      geometry_msgs
    )
  endif()
endif()

ament_package()