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
  uint16_t start;
  int16_t l_speed;
  int16_t r_speed;
  uint16_t checksum;
};

struct __attribute__((packed)) SerialFeedback
{
  uint16_t start;
  int16_t cmd_l;
  int16_t cmd_r;
  int16_t speed_r_meas;
  int16_t speed_l_meas;
  int16_t bat_voltage;
  int16_t board_temp;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  uint16_t quat_w_low;
  int16_t quat_w_high;
  uint16_t quat_x_low;
  int16_t quat_x_high;
  uint16_t quat_y_low;
  int16_t quat_y_high;
  uint16_t quat_z_low;
  int16_t quat_z_high;
  int16_t euler_pitch;
  int16_t euler_roll;
  int16_t euler_yaw;
  int16_t temperature;
  uint16_t sensors;
  uint16_t cmd_led;
  uint16_t checksum;
};
}  // namespace paxi_hardware

#endif  // PAXI_HARDWARE__HOVERBOARD_PROTOCOL_STRUCT_HPP_
