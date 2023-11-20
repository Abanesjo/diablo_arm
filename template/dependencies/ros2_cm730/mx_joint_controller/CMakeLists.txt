cmake_minimum_required(VERSION 3.5)
project(mx_joint_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cm730controller_msgs REQUIRED)
find_package(mx_joint_controller_msgs REQUIRED)

add_library(mx_joint_controller
  src/mx_joint_controller.cpp)

target_include_directories(mx_joint_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(mx_joint_controller
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "cm730controller_msgs"
  "mx_joint_controller_msgs")

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(mx_joint_controller PRIVATE "MX_JOINT_CONTROLLER_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS mx_joint_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(mx_joint_controller_node src/mx_joint_controller_node.cpp)
target_include_directories(mx_joint_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mx_joint_controller_node mx_joint_controller)

install(TARGETS mx_joint_controller_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_mx_joint_controller_tests test/mx_joint_controller_tests.cpp)
  if(TARGET ${PROJECT_NAME}_mx_joint_controller_tests)
    target_include_directories(${PROJECT_NAME}_mx_joint_controller_tests PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    ament_target_dependencies(${PROJECT_NAME}_mx_joint_controller_tests
      "rclcpp"
      "sensor_msgs"
      "cm730controller_msgs"
      "mx_joint_controller_msgs")
  endif()
endif()

ament_export_dependencies(
  "sensor_msgs"
  "mx_joint_controller_msgs"
)
ament_export_include_directories(
  include
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_libraries(
  mx_joint_controller
)

ament_package()
