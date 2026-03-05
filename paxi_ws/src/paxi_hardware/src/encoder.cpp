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

namespace paxi_hardware
{

using paxi_common::hardware_loggers::LOGGER_ENCODER;
using paxi_common::math::RPM_TO_RAD_S;


EncoderKinematics::EncoderKinematics()
: prev_l_rad_per_sec_{0},
  prev_r_rad_per_sec_{0},
  first_read_enc_{true},
  last_read_time_enc_{0}
{}

void EncoderKinematics::update_angular_position(
  const rclcpp::Time & time,
  std::int16_t r_rpm,
  std::int16_t l_rpm,
  std::vector<double> & state_positions)
{
  if (first_read_enc_) {
    prev_l_rad_per_sec_ = l_rpm * RPM_TO_RAD_S;
    prev_r_rad_per_sec_ = r_rpm * RPM_TO_RAD_S;
    last_read_time_enc_ = time;
    first_read_enc_ = false;
    return;
  }

  const double delta_time = time.seconds() - last_read_time_enc_.seconds();
  last_read_time_enc_ = time;

  if (delta_time <= 0.0) {
    RCLCPP_WARN(
      rclcpp::get_logger(LOGGER_ENCODER),
      "Failed to update encoder as delta time is negative or zero [%f]",
      delta_time
    );
    return;
  }

  const double l_rad_per_sec = l_rpm * RPM_TO_RAD_S;
  const double r_rad_per_sec = r_rpm * RPM_TO_RAD_S;

  const double avg_l_rad_per_sec = (prev_l_rad_per_sec_ + l_rad_per_sec) / 2.0;
  const double avg_r_rad_per_sec = (prev_r_rad_per_sec_ + r_rad_per_sec) / 2.0;

  const double delta_l_theta = avg_l_rad_per_sec * delta_time;
  const double delta_r_theta = avg_r_rad_per_sec * delta_time;

  state_positions[to_index(Wheel::LEFT)] += delta_l_theta;
  state_positions[to_index(Wheel::RIGHT)] += delta_r_theta;

  prev_l_rad_per_sec_ = l_rad_per_sec;
  prev_r_rad_per_sec_ = r_rad_per_sec;
}
}  // namespace paxi_hardware
