cmake_minimum_required(VERSION 3.5)
project(cm730driver)

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
find_package(cm730driver_msgs REQUIRED)

add_library(cm730driver
  src/cm730driver.cpp
  src/cm730device.cpp)

target_include_directories(cm730driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(cm730driver
  "rclcpp"
  "cm730driver_msgs")

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(cm730driver PRIVATE "CM730DRIVER_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS cm730driver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(cm730driver_node src/cm730driver_node.cpp)
target_include_directories(cm730driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(cm730driver_node cm730driver)

install(TARGETS cm730driver_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_cm730service_tests test/cm730service_tests.cpp)
  if(TARGET ${PROJECT_NAME}_cm730service_tests)
    target_include_directories(${PROJECT_NAME}_cm730service_tests PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    ament_target_dependencies(${PROJECT_NAME}_cm730service_tests
      "rclcpp")
  endif()
endif()

ament_export_dependencies(
  "cm730driver_msgs"
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  cm730driver
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
