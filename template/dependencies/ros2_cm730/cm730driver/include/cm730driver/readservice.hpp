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

#ifndef CM730DRIVER__READSERVICE_HPP_
#define CM730DRIVER__READSERVICE_HPP_

#include <algorithm>

#include "cm730driver/cm730service.hpp"
#include "cm730driver_msgs/srv/read.hpp"

#define READ_INSTR 2

namespace cm730driver
{

class ReadService : public Cm730Service<READ_INSTR, cm730driver_msgs::srv::Read, ReadService>
{
public:
  using Base = Cm730Service<READ_INSTR, cm730driver_msgs::srv::Read, ReadService>;
  using Read = cm730driver_msgs::srv::Read;

  using Base::Base;

  size_t txPacketSize(const Read::Request & request) override
  {
    (void)request;
    return HEADER_SIZE + 2 + CHECKSUM_SIZE;
  }

  size_t rxPacketSize(const Read::Request & request) override
  {
    return HEADER_SIZE + request.length + CHECKSUM_SIZE;
  }

  uint8_t getDeviceId(const Read::Request & request) override
  {
    return request.device_id;
  }

  void setDataParameters(const Read::Request & request, Packet & packet) override
  {
    packet[ADDR_PARAMETER] = request.address;
    packet[ADDR_PARAMETER + 1] = request.length;
  }

  void handlePacket(
    Packet const & packet,
    Read::Request const & request,
    Read::Response & response) override
  {
    (void)request;
    std::copy(
      std::next(packet.begin(), HEADER_SIZE), std::prev(packet.end(), CHECKSUM_SIZE),
      std::back_inserter(response.data));
  }
};

}  // namespace cm730driver

#endif  // CM730DRIVER__READSERVICE_HPP_
