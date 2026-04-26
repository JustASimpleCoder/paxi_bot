// Copyright 2026 JustASimpleCoder
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

#include "paxi_hardware/imu.hpp"

#include "sensor_msgs/msg/imu.hpp"

#include <gtest/gtest.h>

#include "paxi_common/utils.hpp"

namespace paxi_hardware
{

using paxi_common::utils::to_index;
using paxi_common::utils::Wheel;

class ImuTest : public ::testing::Test
{

protected:
  void SetUp() override {imu_ = std::make_unique<paxi_hardware::ImuProcessing>();}

  std::unique_ptr<paxi_hardware::ImuProcessing> imu_;
};

class ImuSettersTest : public ImuTest
{
};

class ImuAllZeroFeedbackTest : public ImuTest
{
};

class ImuUpdateTest : public ImuTest, public ::testing::WithParamInterface<int>
{
};

class ImuSetLinkNameTest : public ImuTest,
  public ::testing::WithParamInterface<char *>
{
};

class ImuFeedbackTest : public ImuTest,
  public ::testing::WithParamInterface<std::pair<std::int16_t, std::uint16_t>>
{
public:
  const SerialFeedback create_serial_feedback(int16_t signed_val, uint16_t unsigned_val);
};

}  // namespace paxi_hardware
