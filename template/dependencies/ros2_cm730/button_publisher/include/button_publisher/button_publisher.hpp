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

#ifndef BUTTON_PUBLISHER__BUTTON_PUBLISHER_HPP_
#define BUTTON_PUBLISHER__BUTTON_PUBLISHER_HPP_

#include "button_publisher/visibility_control.h"

#include <rclcpp/rclcpp.hpp>
#include <cm730controller_msgs/msg/cm730_info.hpp>
#include <cmath>
#include <string>
#include <buttonpublisher_msg/msg/cm730_button.hpp>

namespace button_publisher
{

class ButtonPublisher : public rclcpp::Node
{
public:
  ButtonPublisher();

  virtual ~ButtonPublisher();

private:
  rclcpp::Subscription<cm730controller_msgs::msg::CM730Info>::SharedPtr sub_;

  rclcpp::Publisher<buttonpublisher_msg::msg::CM730Button>::SharedPtr pub_;

  // Initialise ButtonMsg
  bool is_left_pressed = false;
  bool is_middle_pressed = false;

  bool left_long_press = false;
  bool middle_long_press = false;

  builtin_interfaces::msg::Time time_left_pressed;
  builtin_interfaces::msg::Time time_middle_pressed;

};

}  // namespace button_publisher

#endif  // Button_PUBLISHER__Button_PUBLISHER_HPP_
