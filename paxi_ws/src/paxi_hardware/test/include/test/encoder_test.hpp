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

#include "paxi_hardware/encoder.hpp"

#include <gtest/gtest.h>

#include "paxi_common/utils.hpp"

#include <random>

namespace paxi_hardware
{


class EncoderKinematicsTest : public ::testing::Test
{
protected:
  void SetUp() override {encoder_kin = std::make_unique<paxi_hardware::EncoderKinematics>();}

  std::unique_ptr<paxi_hardware::EncoderKinematics> encoder_kin;
};

class EncoderKinematicsConstRPMTest : public EncoderKinematicsTest,
  public ::testing::WithParamInterface<int>
{
public:
  inline double get_random_time_jump() noexcept {return distribution(rng);}

  void encoder_accumlation_loop_const_time(
    rclcpp::Time & time, std::int16_t rpm_l, std::int16_t rpm_r);

  void encoder_accumlation_loop_random_time(
    rclcpp::Time & time, std::int16_t rpm_l, std::int16_t rpm_r);

private:
  static constexpr double delta_time_change = 0.1;    // in seconds
  static constexpr double num_of_time_deltas = 1000;    // number of discrete test time changes

  // random generate for random time jumps
  static constexpr size_t seed = 42;
  std::mt19937 rng{seed};
  std::uniform_real_distribution<> distribution{0.001, 1};
};

}  // namespace paxi_hardware
