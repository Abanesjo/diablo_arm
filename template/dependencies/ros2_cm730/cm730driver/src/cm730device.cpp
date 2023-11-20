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

#include "cm730driver/cm730device.hpp"

#include <rclcpp/logging.hpp>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>

#include <cstdio>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

namespace cm730driver
{

Cm730Device::Cm730Device(std::string path)
: mPath{std::move(path)},
  mDevice{-1}
{
}

void Cm730Device::open()
{
  auto newtio = termios{};
  auto serinfo = serial_struct{};
  double baudrate = 1000000.0;  // bps (1Mbps)

  // Make sure device is closed before trying to open it
  close();

  if ((mDevice = ::open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY | O_SYNC)) < 0) {
    close();
    throw std::runtime_error("Failed opening CM730");
  }

  // Set IO settings
  // You must set 38400bps in order to be able to set non-standard rate!
  newtio.c_cflag = B38400 | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;
  tcsetattr(mDevice, TCSANOW, &newtio);

  // Reset all serial info
  if (ioctl(mDevice, TIOCGSERIAL, &serinfo) < 0) {
    close();
    throw std::runtime_error("Failed setting baud rate");
  }

  serinfo.flags &= ~ASYNC_SPD_MASK;
  serinfo.flags |= ASYNC_SPD_CUST;
  serinfo.custom_divisor = serinfo.baud_base / baudrate;

  // Set our serial port to use low latency mode (otherwise the USB
  // driver buffers for 16ms before sending data)
  serinfo.flags |= ASYNC_LOW_LATENCY;

  if (ioctl(mDevice, TIOCSSERIAL, &serinfo) < 0) {
    close();
    throw std::runtime_error("Failed setting serial flags");
  }

  // FLush all data received but not read
  tcflush(mDevice, TCIFLUSH);

  RCLCPP_INFO(rclcpp::get_logger("cm730device"), "Successfully opened CM730");
}

void Cm730Device::close()
{
  if (mDevice > 0) {
    if (::close(mDevice) != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("cm730device"), "Failed closing CM730");
      return;
    }
  }
  mDevice = -1;
  RCLCPP_INFO(rclcpp::get_logger("cm730device"), "Successfully closed CM730");
}

void Cm730Device::clear()
{
  tcflush(mDevice, TCIFLUSH);
}

int Cm730Device::write(uint8_t const * outPacket, size_t size)
{
  auto i = ::write(mDevice, outPacket, size);
  if (i < int64_t(size)) {
    RCLCPP_ERROR(rclcpp::get_logger("cm730device"), "Failed writing complete message to CM730");
  } else if (i < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("cm730device"), "Failed writing message to CM730");
  }
  return i;
}

int Cm730Device::read(uint8_t * inPacket, size_t size)
{
  auto i = ::read(mDevice, inPacket, size);
  if (i < 0) {
    switch (errno) {
      case EBADF:
        RCLCPP_ERROR(
          rclcpp::get_logger(
            "cm730device"), "Not a valid file descriptor or is not open for reading");
        break;
      case EFAULT:
        RCLCPP_ERROR(
          rclcpp::get_logger("cm730device"),
          "Buffer outside of accessible address space");
        break;
      case EINTR:
        RCLCPP_ERROR(
          rclcpp::get_logger(
            "cm730device"), "Reading was interrupted by a signal before any data was read");
        break;
      case EINVAL:
        RCLCPP_ERROR(
          rclcpp::get_logger(
            "cm730device"), "File descriptor invalid, or invalid alignment");
        break;
      case EIO:
        RCLCPP_ERROR(rclcpp::get_logger("cm730device"), "I/O error");
        break;
      case EAGAIN:
        // There was no real error, just no data available
        i = 0;
        break;
      default:
        RCLCPP_ERROR(rclcpp::get_logger("cm730device"), "Unknown read error");
    }
  }

  return i;
}

}  // namespace cm730driver
