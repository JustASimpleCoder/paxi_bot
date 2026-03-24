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

#ifndef PAXI_HARDWARE__ENCODER_HPP_
#define PAXI_HARDWARE__ENCODER_HPP_

#include <cstdint>
#include <vector>

#include "paxi_common/hardware_logger_names.hpp"
#include "paxi_common/math.hpp"
#include "paxi_common/utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace paxi_hardware
{
class EncoderKinematics
{
public:
  EncoderKinematics();
  ~EncoderKinematics() = default;

  EncoderKinematics(const EncoderKinematics &) = delete;
  EncoderKinematics & operator=(const EncoderKinematics &) = delete;

  EncoderKinematics(EncoderKinematics &&) noexcept = default;
  EncoderKinematics & operator=(EncoderKinematics &&) noexcept = default;

  void update_angular_position(
    const rclcpp::Time & time, std::int16_t r_rpm, std::int16_t l_rpm,
    std::vector<double> & state_positions);

private:
  double prev_l_rad_per_sec_;
  double prev_r_rad_per_sec_;

  bool first_read_enc_;
  rclcpp::Time last_read_time_enc_;
};
}  // namespace paxi_hardware

#endif  // PAXI_HARDWARE__ENCODER_HPP_
