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

#include "button_publisher/button_publisher.hpp"

#include <string>
#include <memory>

namespace button_publisher
{
ButtonPublisher::ButtonPublisher()
: rclcpp::Node{"button_publisher"}
{
  pub_ = create_publisher<buttonpublisher_msg::msg::CM730Button>(
    "/buttons/data",
    rclcpp::SensorDataQoS());

  sub_ = create_subscription<cm730controller_msgs::msg::CM730Info>(
    "/cm730/cm730info",
    rclcpp::SensorDataQoS(),
    [ = ](cm730controller_msgs::msg::CM730Info::SharedPtr info) {
      auto buttonMsg = std::make_unique<buttonpublisher_msg::msg::CM730Button>();

      // cm730 button's info
      // Left Button = 1 :001
      // Middle Button = 2 : 010
      // Right Button = 4 : 100

      rclcpp::Clock clock;
      rclcpp::Time tl, tm;

      if ((info->dyna.button & 0x01) == 0x01) {

        if (!is_left_pressed) {
          time_left_pressed = clock.now();
          is_left_pressed = true;
        }
      } else {

        //Button released
        if (is_left_pressed) {
          tl = clock.now();

          if ((tl.seconds() - time_left_pressed.sec) > 1.0) {
            left_long_press = true;
          } else {
            left_long_press = false;
          }

          buttonMsg->is_left_pressed = is_left_pressed;
          buttonMsg->left_long_press = left_long_press;
          pub_->publish(std::move(buttonMsg));
        }
        is_left_pressed = false;
      }

      if ((info->dyna.button & 0x02) == 0x02) {

        if (!is_middle_pressed) {
          time_middle_pressed = clock.now();
          is_middle_pressed = true;
        }
      } else {

        //Button released
        if (is_middle_pressed) {
          tm = clock.now();

          if ((tm.seconds() - time_middle_pressed.sec) > 1.0) {
            middle_long_press = true;
          } else {
            middle_long_press = false;
          }

          buttonMsg->is_middle_pressed = is_middle_pressed;
          buttonMsg->middle_long_press = middle_long_press;
          pub_->publish(std::move(buttonMsg));
        }
        is_middle_pressed = false;
      }

    });

}

ButtonPublisher::~ButtonPublisher()
{
}

}
