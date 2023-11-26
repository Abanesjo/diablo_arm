cmake_minimum_required(VERSION 3.5)
project(diablo_gazebo_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(generic_motor_plugin SHARED src/motor_controller.cpp)

target_include_directories(generic_motor_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(generic_motor_plugin
  "gazebo_ros"
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
)

add_library(low_ctrl_plugin SHARED src/low_level_controller.cpp)

target_include_directories(low_ctrl_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(low_ctrl_plugin
  "gazebo_ros"
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
)

add_library(sample_plugin SHARED src/sample_plugin.cpp)
target_include_directories(sample_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(sample_plugin
  "gazebo_ros"
  "std_msgs")

install (TARGETS
  generic_motor_plugin low_ctrl_plugin sample_plugin
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
