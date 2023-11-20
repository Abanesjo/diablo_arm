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

#include "mx_joint_controller/mx_joint_controller.hpp"

#include <string>
#include <vector>
#include <memory>
#include <utility>

namespace mx_joint_controller
{

MxJointController::MxJointController()
: rclcpp::Node{"mx_joint_controller"}
{
  auto jointNames = std::vector<std::string>{};
  get_parameter_or(
    "joint_names", jointNames, {
      "base",
      "shoulder-pitch-r",
      "shoulder-pitch-l",
      "shoulder-roll-r",
      "shoulder-roll-l",
      "elbow-r",
      "elbow-l",

      "hip-yaw-r",
      "hip-yaw-l",
      "hip-roll-r",
      "hip-roll-l",
      "hip-pitch-r",
      "hip-pitch-l",
      "knee-r",
      "knee-l",
      "ankle-pitch-r",
      "ankle-pitch-l",
      "ankle-roll-r",
      "ankle-roll-l",

      "head-pan",
      "head-tilt",
    });

  mx28CommandPub_ = create_publisher<MX28Command>("/cm730/mx28command", rclcpp::ServicesQoS());
  jointStatePub_ = create_publisher<JointState>("/joint_states", rclcpp::SensorDataQoS());

  mx28InfoSub_ = create_subscription<MX28InfoArray>(
    "/cm730/mx28info",
    rclcpp::ServicesQoS(),
    [ = ](MX28InfoArray::SharedPtr info) {
      auto jointStateMsg = std::make_unique<JointState>();
      for (auto const & mx : info->mx28s) {
        jointStateMsg->name.push_back(jointNames[mx.stat.id]);
        jointStateMsg->position.push_back(value2Rads(mx.dyna.present_position));
      }
      jointStateMsg->header = info.get()->header;
      jointStatePub_->publish(std::move(jointStateMsg));
    });

  jointCommandSub_ = create_subscription<JointCommand>(
    "/cm730/joint_commands",
    rclcpp::SensorDataQoS(),
    [ = ](JointCommand::SharedPtr cmd) {
      auto mx28Command = MX28Command{};
      // Transform all names to IDs
      std::transform(
        cmd->name.begin(), cmd->name.end(),
        std::back_inserter(mx28Command.device_id),
        [&](std::string const & name) -> int {
          auto iter = std::find(jointNames.begin(), jointNames.end(), name);
          if (iter == jointNames.end()) {
            RCLCPP_ERROR(get_logger(), "Unknown joint: " + name);
            return -1;
          }
          return std::distance(jointNames.begin(), iter);
        });

      // Turn PID parameters to raw values
      // Transformations from: http://support.robotis.com/en/product/actuator/dynamixel/mx_series/mx-28at_ar.htm#Actuator_Address_1A
      std::transform(
        cmd->p_gain.begin(), cmd->p_gain.end(),
        std::back_inserter(mx28Command.p_gain),
        [ = ](float value) -> uint8_t {
          auto v = unsigned(value * 8);
          if (v > 254) {
            RCLCPP_WARN(get_logger(), "P gain too high: " + std::to_string(value));
            v = 254;
          }
          return v;
        });

      std::transform(
        cmd->i_gain.begin(), cmd->i_gain.end(),
        std::back_inserter(mx28Command.i_gain),
        [ = ](float value) -> uint8_t {
          auto v = unsigned(value * 2048 / 1000);
          if (v > 254) {
            RCLCPP_WARN(get_logger(), "I gain too high: " + std::to_string(value));
            v = 254;
          }
          return v;
        });

      std::transform(
        cmd->d_gain.begin(), cmd->d_gain.end(),
        std::back_inserter(mx28Command.d_gain),
        [ = ](float value) -> uint8_t {
          auto v = unsigned(value * 1000 / 4);
          if (v > 254) {
            RCLCPP_WARN(get_logger(), "D gain too high: " + std::to_string(value));
            v = 254;
          }
          return v;
        });

      // Turn angles in radians to raw values
      std::transform(
        cmd->position.begin(), cmd->position.end(),
        std::back_inserter(mx28Command.goal_position),
        [ = ](float angle) -> uint16_t {
          return rads2Value(angle);
        });

      mx28CommandPub_->publish(mx28Command);
    });
}

MxJointController::~MxJointController()
{
}

}  // namespace mx_joint_controller
