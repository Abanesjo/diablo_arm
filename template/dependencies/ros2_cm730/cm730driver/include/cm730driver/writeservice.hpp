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

#ifndef CM730DRIVER__WRITESERVICE_HPP_
#define CM730DRIVER__WRITESERVICE_HPP_

#include <algorithm>

#include "cm730driver/cm730service.hpp"
#include "cm730driver_msgs/srv/write.hpp"

#define WRITE_INSTR 3

namespace cm730driver
{

class WriteService : public Cm730Service<WRITE_INSTR, cm730driver_msgs::srv::Write, WriteService>
{
public:
  using Base = Cm730Service<WRITE_INSTR, cm730driver_msgs::srv::Write, WriteService>;
  using Write = cm730driver_msgs::srv::Write;

  using Base::Base;

  size_t txPacketSize(const Write::Request & request) override
  {
    return HEADER_SIZE + 1 + request.data.size() + CHECKSUM_SIZE;
  }

  size_t rxPacketSize(const Write::Request & request) override
  {
    (void)request;
    return HEADER_SIZE + CHECKSUM_SIZE;
  }

  uint8_t getDeviceId(const Write::Request & request) override
  {
    return request.device_id;
  }

  void setDataParameters(const Write::Request & request, Packet & packet) override
  {
    packet[ADDR_PARAMETER] = request.address;
    std::copy(
      request.data.begin(), request.data.end(),
      std::next(packet.begin(), ADDR_PARAMETER + 1));
  }

  void handlePacket(
    Packet const & packet,
    Write::Request const & request,
    Write::Response & response) override
  {
    (void)packet;
    (void)request;
    (void)response;
  }
};

}  // namespace cm730driver

#endif  // CM730DRIVER__WRITESERVICE_HPP_
