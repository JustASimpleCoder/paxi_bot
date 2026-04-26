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

#include "test/encoder_test.hpp"

namespace paxi_hardware
{

using paxi_common::utils::to_index;
using paxi_common::utils::Wheel;

void EncoderKinematicsConstRPMTest::encoder_accumlation_loop_const_time(
  rclcpp::Time & time, std::int16_t rpm_l, std::int16_t rpm_r)
{
  std::vector<double> state_positions{0, 0};

  const double omega_r = rpm_r * paxi_common::math::RPM_TO_RAD_S;
  const double omega_l = rpm_l * paxi_common::math::RPM_TO_RAD_S;

  double expected_position_l = 0.0;
  double expected_position_r = 0.0;

  encoder_kin->update_angular_position(time, rpm_r, rpm_l, state_positions);
  EXPECT_DOUBLE_EQ(state_positions[to_index(Wheel::LEFT)], expected_position_l);
  EXPECT_DOUBLE_EQ(state_positions[to_index(Wheel::RIGHT)], expected_position_r);

  for (std::size_t i = 0u; i < num_of_time_deltas; ++i) {
    time += rclcpp::Duration::from_seconds(delta_time_change);
    encoder_kin->update_angular_position(time, rpm_r, rpm_l, state_positions);
    expected_position_l += omega_l * delta_time_change;
    expected_position_r += omega_r * delta_time_change;
  }

  EXPECT_NEAR(state_positions[to_index(Wheel::LEFT)], expected_position_l, 1e-2);
  EXPECT_NEAR(state_positions[to_index(Wheel::RIGHT)], expected_position_r, 1e-2);
}

void EncoderKinematicsConstRPMTest::encoder_accumlation_loop_random_time(
  rclcpp::Time & time, std::int16_t rpm_l, std::int16_t rpm_r)
{
  std::vector<double> state_positions{0, 0};

  const double omega_r = rpm_r * paxi_common::math::RPM_TO_RAD_S;
  const double omega_l = rpm_l * paxi_common::math::RPM_TO_RAD_S;

  double expected_position_l = 0.0;
  double expected_position_r = 0.0;

  encoder_kin->update_angular_position(time, rpm_r, rpm_l, state_positions);
  EXPECT_DOUBLE_EQ(state_positions[to_index(Wheel::LEFT)], expected_position_l);
  EXPECT_DOUBLE_EQ(state_positions[to_index(Wheel::RIGHT)], expected_position_r);

  for (std::size_t i = 0u; i < num_of_time_deltas; ++i) {
    double random_time_jump = get_random_time_jump();

    time += rclcpp::Duration::from_seconds(delta_time_change * random_time_jump);
    encoder_kin->update_angular_position(time, rpm_r, rpm_l, state_positions);
    expected_position_l += (omega_l * delta_time_change * random_time_jump);
    expected_position_r += (omega_r * delta_time_change * random_time_jump);
  }

  EXPECT_NEAR(state_positions[to_index(Wheel::LEFT)], expected_position_l, 1e-2);
  EXPECT_NEAR(state_positions[to_index(Wheel::RIGHT)], expected_position_r, 1e-2);
}

TEST_P(EncoderKinematicsConstRPMTest, UpdateEncoderConstTime)
{
  std::vector<double> state_positions{0, 0};

  const std::int16_t const_rpm = GetParam();
  rclcpp::Time time = rclcpp::Time{0, 0};

  encoder_accumlation_loop_const_time(time, const_rpm, const_rpm);
}

TEST_P(EncoderKinematicsConstRPMTest, UpdateEncoderRandomTime)
{
  std::vector<double> state_positions{0, 0};

  const std::int16_t const_rpm = GetParam();
  rclcpp::Time time = rclcpp::Time{0, 0};

  encoder_accumlation_loop_random_time(time, const_rpm, const_rpm);
}

TEST_P(EncoderKinematicsConstRPMTest, UpdateEncoderSpinningConstTime)
{
  const std::int16_t const_rpm_r = GetParam();
  const std::int16_t const_rpm_l = (const_rpm_r == INT16_MIN) ? INT16_MAX : -const_rpm_r;

  rclcpp::Time time = rclcpp::Time{0, 0};

  encoder_accumlation_loop_const_time(time, const_rpm_l, const_rpm_r);
}

TEST_P(EncoderKinematicsConstRPMTest, UpdateEncoderSpinningNonConstTime)
{
  const std::int16_t const_rpm_r = GetParam();
  const std::int16_t const_rpm_l = (const_rpm_r == INT16_MIN) ? INT16_MAX : -const_rpm_r;

  rclcpp::Time time = rclcpp::Time{0, 0};
  encoder_accumlation_loop_random_time(time, const_rpm_l, const_rpm_r);
}

INSTANTIATE_TEST_SUITE_P(
  UpdateEncoderTest, EncoderKinematicsConstRPMTest,
  ::testing::Values(
    INT16_MIN, -10000, -5000, -500, -100, -50, -5, -1, 0, 1, 5, 50, 100, 500, 5000,
    10000, INT16_MAX));

}  // namespace paxi_hardware
