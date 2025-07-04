cmake_minimum_required(VERSION 3.8...3.27)
project(gui_pose_issuer)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# Include directories
include_directories(include)

# Enable Qt5 MOC, UIC, and RCC processing
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Create a library from the class implementation
add_library(${PROJECT_NAME}_lib
  src/gui_pose_issuer.cpp
)

# Link dependencies to library
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  geometry_msgs
)

target_link_libraries(${PROJECT_NAME}_lib
  Qt5::Widgets
#   Qt5::Test
)

# Create executable
add_executable(gui_pose_node
  src/gui_pose_issuer_node.cpp
)

# Link library to executable
target_link_libraries(gui_pose_node ${PROJECT_NAME}_lib Qt5::Widgets)

# Link dependencies to executable
ament_target_dependencies(gui_pose_node
  rclcpp
  geometry_msgs
)

# Install executable
install(TARGETS
  gui_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(rclcpp geometry_msgs)

# Testing -- commenting for now because it's buggy
# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)

#   ament_add_gtest(test_gui_pose_issuer test/test_gui_pose_issuer.cpp)
#   if(TARGET test_gui_pose_issuer)
#     target_link_libraries(test_gui_pose_issuer ${PROJECT_NAME}_lib)
#     ament_target_dependencies(test_gui_pose_issuer
#       rclcpp
#       geometry_msgs
#     )
#   endif()
# endif()

ament_package()
