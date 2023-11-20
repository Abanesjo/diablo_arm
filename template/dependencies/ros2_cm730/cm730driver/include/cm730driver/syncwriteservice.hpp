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

#ifndef CM730DRIVER__SYNCWRITESERVICE_HPP_
#define CM730DRIVER__SYNCWRITESERVICE_HPP_

#include <algorithm>

#include "cm730driver/cm730service.hpp"
#include "cm730driver_msgs/srv/sync_write.hpp"

#define SYNC_WRITE_INSTR 131

namespace cm730driver
{
class SyncWriteService : public Cm730Service<SYNC_WRITE_INSTR, cm730driver_msgs::srv::SyncWrite,
    SyncWriteService>
{
public:
  using Base = Cm730Service<SYNC_WRITE_INSTR, cm730driver_msgs::srv::SyncWrite, SyncWriteService>;
  using SyncWrite = cm730driver_msgs::srv::SyncWrite;

  using Base::Base;

  size_t txPacketSize(const SyncWrite::Request & request) override
  {
    auto dataSize = std::accumulate(
      request.write_requests.begin(), request.write_requests.end(), 0u,
      [](auto sum, auto element) {return sum + element.data.size() + 1;});

    return HEADER_SIZE + 2 + dataSize + CHECKSUM_SIZE;
  }

  size_t rxPacketSize(const SyncWrite::Request & request) override
  {
    (void)request;
    return 0;
  }

  uint8_t getDeviceId(const SyncWrite::Request & request) override
  {
    (void)request;
    return 254;
  }

  void setDataParameters(const SyncWrite::Request & request, Packet & packet) override
  {
    packet[ADDR_PARAMETER] = request.address;
    packet[ADDR_PARAMETER + 1] = request.length;

    auto cursor = ADDR_PARAMETER + 2;
    for (auto const & writeRequest : request.write_requests) {
      packet[cursor++] = writeRequest.device_id;
      std::copy(
        writeRequest.data.begin(), writeRequest.data.end(),
        std::next(packet.begin(), cursor));
      cursor += request.length;
    }
  }

  void handlePacket(
    Packet const & packet,
    const SyncWrite::Request & request,
    SyncWrite::Response & response) override
  {
    (void)packet;
    (void)request;
    (void)response;
  }
};
}  // namespace cm730driver

#endif  // CM730DRIVER__SYNCWRITESERVICE_HPP_
