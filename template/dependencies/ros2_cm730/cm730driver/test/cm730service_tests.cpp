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

#include <gtest/gtest.h>

#include "cm730driver/cm730service.hpp"

using cm730driver::Cm730ServiceBase;

TEST(Cm730ServiceTests, initPacket) {
  auto packet = Cm730ServiceBase::initPacket(10, 200, 2);

  ASSERT_EQ(10u, packet.size());
  ASSERT_EQ(0xFF, packet[0]);
  ASSERT_EQ(0xFF, packet[1]);
  ASSERT_EQ(200, packet[2]);
  ASSERT_EQ(6, packet[3]);
  ASSERT_EQ(2, packet[4]);
}

TEST(Cm730ServiceTests, calcCheckSum) {
  auto packet = Cm730ServiceBase::Packet{
    0xFF,
    0xFF,
    200,
    2,
    1,
    0,
  };

  ASSERT_EQ(52, Cm730ServiceBase::calcChecksum(packet));
  Cm730ServiceBase::setChecksum(packet);
  ASSERT_EQ(52, packet[5]);
  // Having checksum set should not matter
  ASSERT_EQ(52, Cm730ServiceBase::calcChecksum(packet));
}

TEST(Cm730ServiceTests, checkChecksum) {
  auto packet = Cm730ServiceBase::Packet{
    0xFF,
    0xFF,
    200,
    2,
    1,
    0,
  };

  ASSERT_FALSE(Cm730ServiceBase::checkChecksum(packet));
  Cm730ServiceBase::setChecksum(packet);
  ASSERT_TRUE(Cm730ServiceBase::checkChecksum(packet));
}
