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

#ifndef PAXI_HARDWARE__HOVERBOARD_PROTOCOL_STRUCT_HPP_
#define PAXI_HARDWARE__HOVERBOARD_PROTOCOL_STRUCT_HPP_

#include <cstdint>
#include <utility>

namespace paxi_hardware
{
struct __attribute__((packed)) SerialCommand
{
  std::uint16_t start;
  std::int16_t l_speed;
  std::int16_t r_speed;
  std::uint16_t checksum;
};

struct __attribute__((packed)) SerialFeedback
{
  std::uint16_t start;
  std::int16_t cmd_l;
  std::int16_t cmd_r;
  std::int16_t speed_r_meas;
  std::int16_t speed_l_meas;
  std::int16_t bat_voltage;
  std::int16_t board_temp;
  std::int16_t gyro_x;
  std::int16_t gyro_y;
  std::int16_t gyro_z;
  std::int16_t accel_x;
  std::int16_t accel_y;
  std::int16_t accel_z;
  std::uint16_t quat_w_low;
  std::int16_t quat_w_high;
  std::uint16_t quat_x_low;
  std::int16_t quat_x_high;
  std::uint16_t quat_y_low;
  std::int16_t quat_y_high;
  std::uint16_t quat_z_low;
  std::int16_t quat_z_high;
  std::int16_t euler_pitch;
  std::int16_t euler_roll;
  std::int16_t euler_yaw;
  std::int16_t temperature;
  std::uint16_t sensors;
  std::uint16_t cmd_led;
  std::uint16_t checksum;
};
}  // namespace paxi_hardware

#endif  // PAXI_HARDWARE__HOVERBOARD_PROTOCOL_STRUCT_HPP_
