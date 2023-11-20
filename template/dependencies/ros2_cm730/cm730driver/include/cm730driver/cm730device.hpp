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

#ifndef CM730DRIVER__CM730DEVICE_HPP_
#define CM730DRIVER__CM730DEVICE_HPP_

#include <cstdint>
#include <cstring>
#include <string>

namespace cm730driver
{

class Cm730Device
{
public:
  explicit Cm730Device(std::string path);

  void open();
  void close();

  void clear();

  int write(uint8_t const * outPacket, size_t size);
  int read(uint8_t * inPacket, size_t size);

private:
  std::string mPath;
  int mDevice;
};

}  // namespace cm730driver

#endif  // CM730DRIVER__CM730DEVICE_HPP_
