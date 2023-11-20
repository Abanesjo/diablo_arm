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

#ifndef MX_JOINT_CONTROLLER__MX_JOINT_CONTROLLER_HPP_
#define MX_JOINT_CONTROLLER__MX_JOINT_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <cm730controller_msgs/msg/mx28_info_array.hpp>
#include <cm730controller_msgs/msg/mx28_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mx_joint_controller_msgs/msg/joint_command.hpp>

#include <cmath>

#include "mx_joint_controller/visibility_control.h"

namespace mx_joint_controller
{

class MxJointController : public rclcpp::Node
{
public:
  MxJointController();

  virtual ~MxJointController();

  static double value2Rads(uint16_t value)
  {
    return (static_cast<int>(value) - 0x0800) * (2 * M_PI) / 4096.0;
  }

  static uint16_t rads2Value(float rads)
  {
    return std::round(rads * 4096.0 / (2 * M_PI) ) + 0x800;
  }

private:
  using MX28InfoArray = cm730controller_msgs::msg::MX28InfoArray;
  using MX28Command = cm730controller_msgs::msg::MX28Command;

  using JointState = sensor_msgs::msg::JointState;
  using JointCommand = mx_joint_controller_msgs::msg::JointCommand;

  rclcpp::Subscription<MX28InfoArray>::SharedPtr mx28InfoSub_;
  rclcpp::Publisher<MX28Command>::SharedPtr mx28CommandPub_;

  rclcpp::Publisher<JointState>::SharedPtr jointStatePub_;
  rclcpp::Subscription<JointCommand>::SharedPtr jointCommandSub_;
};

}  // namespace mx_joint_controller

#endif  // MX_JOINT_CONTROLLER__MX_JOINT_CONTROLLER_HPP_
