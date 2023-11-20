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

#ifndef CM730DRIVER__CM730SERVICE_HPP_
#define CM730DRIVER__CM730SERVICE_HPP_

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <algorithm>
#include <array>
#include <vector>
#include <numeric>
#include <memory>
#include <sstream>
#include <tuple>
#include <string>
#include <utility>

#include "cm730driver/cm730device.hpp"

namespace cm730driver
{
class Cm730Device;

class Cm730ServiceBase
{
public:
  static constexpr uint8_t HEADER_SIZE = 5;
  static constexpr uint8_t CHECKSUM_SIZE = 1;
  static constexpr uint8_t ERROR_SIZE = 1;

  /// Indexes to parts in CM730 packets
  enum  PacketAddr : uint8_t
  {
    ADDR_ID = 2,
    ADDR_LENGTH = 3,
    ADDR_INSTRUCTION = 4,
    ADDR_PARAMETER = 5,    // For TX packet
    ADDR_ERROR = 4,        // For RX packet
    ADDR_DATA = 5,         // For RX packet
  };

  enum ErrorFlag : uint16_t
  {
    VOLTAGE_ERROR = 0x01,
    ANGLE_LIMIT_ERROR = 0x02,
    OVERHEATING_ERROR = 0x04,
    RANGE_ERROR = 0x08,
    CHECKSUM_ERROR = 0x10,
    OVERLOAD_ERROR = 0x20,
    INSTRUCTION_ERROR = 0x40,
    TIMEOUT_ERROR = 0x0100,
    CORRUPT_RESPONSE_ERROR = 0x0200,
  };

  /// Packet data
  using Packet = std::vector<uint8_t>;

  /// Prepare packet header data
  static Packet initPacket(size_t size, uint8_t deviceId, uint8_t instruction);

  /// Set checksum of a prepared packet
  static void setChecksum(Packet & packet);

  /// Calculate checksum of a prepared packet
  static uint8_t calcChecksum(Packet const & packet);

  /// Check whether the checsum of a packet is correct
  static bool checkChecksum(Packet const & packet);
};

/**CM730 Service base class
 *
 * Provides standard methods for any service that transmits and
 * receives messages to and from a CM730.
 */
template<uint8_t INSTR, class ServiceT, class Derived, bool CHECK_CHECKSUM = true>
class Cm730Service : public Cm730ServiceBase
{
public:
  Cm730Service(std::shared_ptr<Cm730Device> device, std::shared_ptr<rclcpp::Clock> clock);

  void handle(
    std::shared_ptr<typename ServiceT::Request> request,
    std::shared_ptr<typename ServiceT::Response> response);

  /** Size of packets to send to CM730
   *
   * Must include header and checksum
   */
  virtual size_t txPacketSize(const typename ServiceT::Request & request) = 0;

  /** Size of packets received from CM730
   *
   * Must include header and checksum
   */
  virtual size_t rxPacketSize(const typename ServiceT::Request & request) = 0;

  /** Device ID for packet
   *
   * @param request Service request that can be used to determine device ID
   */
  virtual uint8_t getDeviceId(const typename ServiceT::Request & request) = 0;

  /** Set parameter bytes
   *
   * @param request Service request that can be used to determine parameters
   * @param packet Packet to send, with header initialised
   */
  virtual void setDataParameters(const typename ServiceT::Request & request, Packet & packet) = 0;

  virtual void handlePacket(
    Packet const & packet,
    const typename ServiceT::Request & request,
    typename ServiceT::Response & response) = 0;

  /** Factory method to create service implementation
   *
   * Creates service object, along with its ROS server
   */
  static std::tuple<std::shared_ptr<Derived>, typename rclcpp::Service<ServiceT>::SharedPtr>
  create(
    rclcpp::Node & node, std::string const & serviceName,
    std::shared_ptr<Cm730Device> device, std::shared_ptr<rclcpp::Clock> clock);

  /// Prepare packet header data
  static Packet initPacket(size_t size, uint8_t deviceId);

private:
  std::shared_ptr<Cm730Device> mDevice;
  std::shared_ptr<rclcpp::Clock> mClock;
};


template<uint8_t INSTR, class ServiceT, class Derived, bool CHECK_CHECKSUM>
Cm730Service<INSTR, ServiceT, Derived, CHECK_CHECKSUM>::Cm730Service(
  std::shared_ptr<Cm730Device> device,
  std::shared_ptr<rclcpp::Clock> clock)
: mDevice{std::move(device)},
  mClock{std::move(clock)}
{}

template<uint8_t INSTR, class ServiceT, class Derived, bool CHECK_CHECKSUM>
void Cm730Service<INSTR, ServiceT, Derived, CHECK_CHECKSUM>::handle(
  std::shared_ptr<typename ServiceT::Request> request,
  std::shared_ptr<typename ServiceT::Response> response)
{
#if (RCLCPP_LOG_MIN_SEVERITY <= RCLCPP_LOG_MIN_SEVERITY_DEBUG)
  {
    auto str = std::ostringstream{};
    str <<
      "Received request: " <<
      rosidl_generator_traits::data_type<typename ServiceT::Request>();
    RCLCPP_DEBUG(rclcpp::get_logger("cm730service"), str.str());
  }
#endif

  // Flush any unread bytes
  mDevice->clear();

  // Prepare packet to send
  auto txPacket = initPacket(txPacketSize(*request), getDeviceId(*request));
  setDataParameters(*request, txPacket);
  setChecksum(txPacket);

#if (RCLCPP_LOG_MIN_SEVERITY <= RCLCPP_LOG_MIN_SEVERITY_DEBUG)
  {
    auto str = std::ostringstream{};
    str << "Writing: ";
    for (auto i = 0u; i < txPacket.size(); ++i) {
      str << int{txPacket[i]} << " ";
    }
    RCLCPP_DEBUG(rclcpp::get_logger("cm730service"), str.str());
  }
#endif

  // Send packet
  mDevice->write(txPacket.data(), txPacket.size());

  // Don't read if no reply is expected
  auto rxSize = rxPacketSize(*request);
  if (rxSize == 0) {
    return;
  }

  // Prepare for reading response
  auto rxPacket = Packet(rxSize);

  auto readStartTime = mClock->now();
  auto nRead = 0u;
  while (nRead < rxPacket.size()) {
    RCLCPP_DEBUG(rclcpp::get_logger("cm730service"), "Starting read loop");
    // Check if we have timed out and give up
    auto readingTime = mClock->now() - readStartTime;
    auto delta_ms = readingTime.nanoseconds() / 1e6;
    if (delta_ms > 15) {
      RCLCPP_ERROR(rclcpp::get_logger("cm730service"), "Timed out");
      response->error = ErrorFlag::TIMEOUT_ERROR;
      return;
    } else if (delta_ms > 12.5) {
      RCLCPP_WARN(rclcpp::get_logger("cm730service"), "Read time exceeds 12.5ms");
    }

    // Read as many bytes as are available, up to how many are still missing
    auto n = mDevice->read(rxPacket.data() + nRead, rxPacket.size() - nRead);
    if (n > 0) {
      // A positive amount of bytes is good
      nRead += n;

      // Shift bytes if first bytes are not equal to the header
      auto headerStart = std::adjacent_find(
        rxPacket.begin(), std::next(rxPacket.begin(), nRead),
        [](uint8_t a, uint8_t b) {return a == 0xFF && b == 0xFF;});
      if (headerStart != rxPacket.begin() && headerStart != rxPacket.end()) {
        std::copy(headerStart, std::next(rxPacket.begin(), nRead), rxPacket.begin());
        nRead -= std::distance(rxPacket.begin(), headerStart);
      }

#if (RCLCPP_LOG_MIN_SEVERITY <= RCLCPP_LOG_MIN_SEVERITY_DEBUG)
      // Log what we've read so far
      {
        auto str = std::ostringstream{};
        str << "Total read: " << nRead << " - ";
        for (auto i = 0u; i < nRead; ++i) {
          str << int{rxPacket[i]} << " ";
        }
        RCLCPP_DEBUG(rclcpp::get_logger("cm730service"), str.str());
      }
#endif
    }

    rclcpp::sleep_for(std::chrono::microseconds{100});
  }

  // Packet fully read, set timestamp
  auto now = mClock->now();
  response->header.stamp = now;

  // Check checksum
  if (CHECK_CHECKSUM && !checkChecksum(rxPacket)) {
    auto str = std::ostringstream{};
    str << "Corrupt packet, invalid checksum: " << nRead << " - ";
    for (auto i = 0u; i < nRead; ++i) {
      str << int{rxPacket[i]} << " ";
    }
    RCLCPP_ERROR(rclcpp::get_logger("cm730service"), str.str());
    response->error = ErrorFlag::CORRUPT_RESPONSE_ERROR;
    return;
  }

  // Check error byte
  if (rxPacket[ADDR_ERROR] != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("cm730service"),
      "Received error byte: " + std::to_string(rxPacket[ADDR_ERROR]));
    response->error = rxPacket[ADDR_ERROR];
    return;
  }

  // Successfully read full packet, create response
  handlePacket(rxPacket, *request, *response);
}

template<uint8_t INSTR, class ServiceT, class Derived, bool CHECK_CHECKSUM>
typename Cm730Service<INSTR, ServiceT, Derived, CHECK_CHECKSUM>::Packet Cm730Service<INSTR,
  ServiceT,
  Derived, CHECK_CHECKSUM>::initPacket(size_t size, uint8_t deviceId)
{
  return Cm730ServiceBase::initPacket(size, deviceId, INSTR);
}

Cm730ServiceBase::Packet Cm730ServiceBase::initPacket(
  size_t size, uint8_t deviceId,
  uint8_t instruction)
{
  auto data = Packet(size);
  data[0] = data[1] = 0xFF;
  data[ADDR_ID] = deviceId;
  data[ADDR_LENGTH] = size - (uint8_t)ADDR_INSTRUCTION;
  data[ADDR_INSTRUCTION] = instruction;
  return data;
}

void Cm730ServiceBase::setChecksum(Packet & data)
{
  *std::prev(data.end(), 1) = calcChecksum(data);
}

uint8_t Cm730ServiceBase::calcChecksum(Packet const & data)
{
  auto sum = std::accumulate(std::next(data.begin(), 2), std::prev(data.end(), 1), 0u);
  return ~sum;
}

bool Cm730ServiceBase::checkChecksum(const Packet & packet)
{
  return packet.back() == calcChecksum(packet);
}

template<uint8_t INSTR, class ServiceT, class Derived, bool CHECK_CHECKSUM>
std::tuple<std::shared_ptr<Derived>, typename rclcpp::Service<ServiceT>::SharedPtr>
Cm730Service<INSTR, ServiceT, Derived, CHECK_CHECKSUM>::create(
  rclcpp::Node & node, std::string const & serviceName,
  std::shared_ptr<Cm730Device> device, std::shared_ptr<rclcpp::Clock> clock)
{
  auto cm730Service = std::make_shared<Derived>(std::move(device), std::move(clock));
  auto rclcppService = node.create_service<ServiceT>(
    serviceName,
    [ = ](std::shared_ptr<typename ServiceT::Request> request,
    std::shared_ptr<typename ServiceT::Response> response) {
      cm730Service->handle(request, response);
    });

  return std::make_tuple(cm730Service, rclcppService);
}

}  // namespace cm730driver

#endif  // CM730DRIVER__CM730SERVICE_HPP_
