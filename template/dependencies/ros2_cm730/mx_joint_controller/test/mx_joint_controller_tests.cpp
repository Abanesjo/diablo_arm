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

#include "mx_joint_controller/mx_joint_controller.hpp"

using mx_joint_controller::MxJointController;

TEST(MxJointControllerTests, value2Rads) {
  ASSERT_EQ(0.0, MxJointController::value2Rads(0x800));
  ASSERT_EQ(-M_PI, MxJointController::value2Rads(0));
  ASSERT_EQ(M_PI, MxJointController::value2Rads(0x1000));
}

TEST(MxJointControllerTests, rads2Value) {
  ASSERT_EQ(0x800, MxJointController::rads2Value(0.0));
  ASSERT_EQ(0, MxJointController::rads2Value(-M_PI));
  ASSERT_EQ(0x1000, MxJointController::rads2Value(M_PI));
}
