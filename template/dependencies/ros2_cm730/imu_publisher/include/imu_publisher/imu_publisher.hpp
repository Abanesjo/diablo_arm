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

#ifndef IMU_PUBLISHER__IMU_PUBLISHER_HPP_
#define IMU_PUBLISHER__IMU_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <cm730controller_msgs/msg/cm730_info.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <cmath>
#include <string>

#include "imu_publisher/visibility_control.h"

namespace imu_publisher
{

class IMUPublisher : public rclcpp::Node
{
public:
  IMUPublisher();

  virtual ~IMUPublisher();

private:
  rclcpp::Subscription<cm730controller_msgs::msg::CM730Info>::SharedPtr sub_;

  std::string imu_frame_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;

  inline auto clamp(double val, double min, double max)->double
  {
    return val<min ? min : val> max ? max : val;
  }

  double gyroValueToRPS(int value);
  double accelToMS2(int value);
};

}  // namespace imu_publisher

#endif  // IMU_PUBLISHER__IMU_PUBLISHER_HPP_
