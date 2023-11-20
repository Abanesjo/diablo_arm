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

#include "imu_publisher/imu_publisher.hpp"

#include <string>
#include <memory>
#include <utility>

namespace imu_publisher
{

IMUPublisher::IMUPublisher()
: rclcpp::Node{"imu_publisher"}
{
  get_parameter_or("imu_frame", imu_frame_, std::string("base_link"));

  pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", rclcpp::SensorDataQoS());

  sub_ = create_subscription<cm730controller_msgs::msg::CM730Info>(
    "/cm730/cm730info",
    rclcpp::SensorDataQoS(),
    [ = ](cm730controller_msgs::msg::CM730Info::SharedPtr info) {
      auto imuStateMsg = std::make_unique<sensor_msgs::msg::Imu>();

      imuStateMsg->header.frame_id = imu_frame_;

      // CM-730 coordinate systems (axes pointing in positiv direction)
      // accelerometer: x right, y forward,  z up (right-handed)
      // ros2: x forward, y left, z up (right-handed)
      // (more http://www.ros.org/reps/rep-0103.html#coordinate-frame-conventions)

      // accelerometer
      imuStateMsg->linear_acceleration.x = accelToMS2(info->dyna.accel.at(1));      // y
      imuStateMsg->linear_acceleration.y = -accelToMS2(info->dyna.accel.at(0));     // x
      imuStateMsg->linear_acceleration.z = accelToMS2(info->dyna.accel.at(2));      // z

      // from cm730
      // x/y counterclockwise, z clockwise
      // gyro
      imuStateMsg->angular_velocity.x = -gyroValueToRPS(info->dyna.gyro.at(0));     // x
      imuStateMsg->angular_velocity.y = -gyroValueToRPS(info->dyna.gyro.at(1));     // y
      imuStateMsg->angular_velocity.z = gyroValueToRPS(info->dyna.gyro.at(2));      // z

      // orientation estimate is unknown at this stage
      // set element at 0 to -1
      // https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Imu.msg
      imuStateMsg->orientation_covariance.at(0) = -1;

      imuStateMsg->header = info->header;
      pub_->publish(std::move(imuStateMsg));
    });
}

IMUPublisher::~IMUPublisher()
{
}

double IMUPublisher::accelToMS2(int value)
{
  // Milli Gs per digit
  static auto accelMgsPerDigit = 8.0;
  // Max measurable Gs
  static auto accelGRange = 4.0;
  // Range raw value
  static auto accelDigitalRange = 1024;

  auto gs = (int{value} - accelDigitalRange / 2) * accelMgsPerDigit / 1000;
  auto clampedGs = clamp(gs, -accelGRange, accelGRange);

  return clampedGs * 9.80665;
}

double IMUPublisher::gyroValueToRPS(int value)
{
  // Milli degrees per second per digit
  const auto gyroMdpsPerDigit = 448.0;
  // Max measurable degrees per second
  const auto gyroDpsRange = 200.0;
  // Range raw value
  const auto gyroDigitalRange = 1024;

  auto degPerSec = (int{value} - gyroDigitalRange / 2) * gyroMdpsPerDigit / 1000;
  auto clampedDegPerSec = clamp(degPerSec, -gyroDpsRange, gyroDpsRange);

  return clampedDegPerSec / 180 * M_PI;
}

}  // namespace imu_publisher
