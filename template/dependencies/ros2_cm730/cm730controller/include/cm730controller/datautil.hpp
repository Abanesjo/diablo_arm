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

#ifndef CM730CONTROLLER__DATAUTIL_HPP_
#define CM730CONTROLLER__DATAUTIL_HPP_

#include <cstdint>
#include <vector>

#include "cm730controller/cm730table.hpp"

namespace cm730controller
{

class DataUtil
{
public:
  template<typename TEnum>
  static uint8_t getByte(std::vector<uint8_t> const & data, TEnum addr, TEnum startAddr)
  {
    return data[uint8_t(addr) - uint8_t(startAddr)];
  }

  template<typename TEnum>
  static uint16_t getWord(std::vector<uint8_t> const & data, TEnum addr, TEnum startAddr)
  {
    auto addr_ = uint8_t(addr) - uint8_t(startAddr);
    return uint16_t{data[addr_]} | (uint16_t{data[addr_ + 1]} << 8);
  }

  template<typename TIter, typename TEnum>
  static void setByte(uint8_t value, TIter & dataIter, TEnum addr, TEnum startAddr)
  {
    auto addr_ = uint8_t(addr) - uint8_t(startAddr);
    *std::next(dataIter, addr_) = value & 0xFF;
  }

  template<typename TIter, typename TEnum>
  static void setWord(uint16_t value, TIter & dataIter, TEnum addr, TEnum startAddr)
  {
    auto addr_ = uint8_t(addr) - uint8_t(startAddr);
    *std::next(dataIter, addr_) = value & 0xFF;
    *std::next(dataIter, addr_ + 1) = (value >> 8) & 0xFF;
  }
};

}  // namespace cm730controller

#endif  // CM730CONTROLLER__DATAUTIL_HPP_
