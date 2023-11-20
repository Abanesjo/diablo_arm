// Copyright 2019 Bold Hearts
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cm730driver/cm730driver.hpp>
#include <cm730controller/cm730controller.hpp>
#include <mx_joint_controller/mx_joint_controller.hpp>
#include <imu_publisher/imu_publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec{};

  auto cmy730DriverNode = std::make_shared<cm730driver::Cm730Driver>();
  auto cm730ControllerNode = std::make_shared<cm730controller::Cm730Controller>();
  auto jointControllerNode = std::make_shared<mx_joint_controller::MxJointController>();
  auto imuPublisherNode = std::make_shared<imu_publisher::IMUPublisher>();

  exec.add_node(cmy730DriverNode);
  exec.add_node(cm730ControllerNode);
  exec.add_node(jointControllerNode);
  exec.add_node(imuPublisherNode);

  exec.spin();

  rclcpp::shutdown();
  cmy730DriverNode = nullptr;
  cm730ControllerNode = nullptr;
  jointControllerNode = nullptr;
  imuPublisherNode = nullptr;

  return 0;
}
