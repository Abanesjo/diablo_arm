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

#ifndef CM730DRIVER__CM730DRIVER_HPP_
#define CM730DRIVER__CM730DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <tuple>

#include "cm730driver_msgs/srv/ping.hpp"
#include "cm730driver_msgs/srv/read.hpp"
#include "cm730driver_msgs/srv/write.hpp"
#include "cm730driver_msgs/srv/bulk_read.hpp"
#include "cm730driver_msgs/srv/sync_write.hpp"

#include "cm730driver/visibility_control.h"

namespace cm730driver
{

class Cm730Device;
class PingService;
class ReadService;
class WriteService;
class BulkReadService;
class SyncWriteService;

class Cm730Driver : public rclcpp::Node
{
public:
  Cm730Driver();

  virtual ~Cm730Driver();

private:
  std::shared_ptr<Cm730Device> mDevice;

  std::tuple<std::shared_ptr<PingService>,
    rclcpp::Service<cm730driver_msgs::srv::Ping>::SharedPtr> mPingServer;

  std::tuple<std::shared_ptr<ReadService>,
    rclcpp::Service<cm730driver_msgs::srv::Read>::SharedPtr> mReadServer;

  std::tuple<std::shared_ptr<WriteService>,
    rclcpp::Service<cm730driver_msgs::srv::Write>::SharedPtr> mWriteServer;

  std::tuple<std::shared_ptr<BulkReadService>,
    rclcpp::Service<cm730driver_msgs::srv::BulkRead>::SharedPtr> mBulkReadServer;

  std::tuple<std::shared_ptr<SyncWriteService>,
    rclcpp::Service<cm730driver_msgs::srv::SyncWrite>::SharedPtr> mSyncWriteServer;
};

}  // namespace cm730driver

#endif  // CM730DRIVER__CM730DRIVER_HPP_
