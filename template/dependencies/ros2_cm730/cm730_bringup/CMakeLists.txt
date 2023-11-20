cmake_minimum_required(VERSION 3.5)
project(cm730_bringup)

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

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cm730driver REQUIRED)
find_package(cm730controller REQUIRED)
find_package(mx_joint_controller REQUIRED)
find_package(imu_publisher REQUIRED)

add_executable(cm730_composed src/cm730_composed.cpp)
target_include_directories(cm730_composed PUBLIC
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(cm730_composed
  "rclcpp"
  "cm730driver"
  "cm730controller"
  "mx_joint_controller"
  "imu_publisher")

install(TARGETS cm730_composed
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
