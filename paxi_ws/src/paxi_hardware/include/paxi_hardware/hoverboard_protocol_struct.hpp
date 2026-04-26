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

  SerialCommand()
  : start{0}, l_speed{0}, r_speed{0}, checksum{0}
  {}

  SerialCommand(
    std::uint16_t start, std::int16_t l_speed, std::int16_t r_speed, std::uint16_t checksum)
  : start{start}, l_speed{l_speed}, r_speed{r_speed}, checksum{checksum}
  {}

  SerialCommand & operator=(SerialCommand & other)
  {
    if (this != &other) {
      start = other.start;
      l_speed = other.l_speed;
      r_speed = other.r_speed;
      checksum = other.checksum;
    }
    return *this;
  }

  SerialCommand(SerialCommand & other)
  : start{other.start}, l_speed{other.l_speed}, r_speed{other.r_speed}, checksum{other.checksum}
  {}
  SerialCommand(SerialCommand && other)
  : start{other.start}, l_speed{other.l_speed}, r_speed{other.r_speed}, checksum{other.checksum}
  {}
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

  SerialFeedback()
  : start{0},
    cmd_l{0},
    cmd_r{0},
    speed_r_meas{0},
    speed_l_meas{0},
    bat_voltage{0},
    board_temp{0},
    gyro_x{0},
    gyro_y{0},
    gyro_z{0},
    accel_x{0},
    accel_y{0},
    accel_z{0},
    quat_w_low{0},
    quat_w_high{0},
    quat_x_low{0},
    quat_x_high{0},
    quat_y_low{0},
    quat_y_high{0},
    quat_z_low{0},
    quat_z_high{0},
    euler_pitch{0},
    euler_roll{0},
    euler_yaw{0},
    temperature{0},
    sensors{0},
    cmd_led{0},
    checksum{0}
  {}

  SerialFeedback(
    std::uint16_t start, std::int16_t cmd_l, std::int16_t cmd_r, std::int16_t speed_r_meas,
    std::int16_t speed_l_meas, std::int16_t bat_voltage, std::int16_t board_temp,
    std::int16_t gyro_x, std::int16_t gyro_y, std::int16_t gyro_z,
    std::int16_t accel_x, std::int16_t accel_y, std::int16_t accel_z,
    std::uint16_t quat_w_low, std::int16_t quat_w_high,
    std::uint16_t quat_x_low, std::int16_t quat_x_high,
    std::uint16_t quat_y_low, std::int16_t quat_y_high,
    std::uint16_t quat_z_low, std::int16_t quat_z_high,
    std::int16_t euler_pitch, std::int16_t euler_roll, std::int16_t euler_yaw,
    std::int16_t temperature, std::uint16_t sensors, std::uint16_t cmd_led,
    std::uint16_t checksum)
  : start{start},
    cmd_l{cmd_l},
    cmd_r{cmd_r},
    speed_r_meas{speed_r_meas},
    speed_l_meas{speed_l_meas},
    bat_voltage{bat_voltage},
    board_temp{board_temp},
    gyro_x{gyro_x},
    gyro_y{gyro_y},
    gyro_z{gyro_z},
    accel_x{accel_x},
    accel_y{accel_y},
    accel_z{accel_z},
    quat_w_low{quat_w_low},
    quat_w_high{quat_w_high},
    quat_x_low{quat_x_low},
    quat_x_high{quat_x_high},
    quat_y_low{quat_y_low},
    quat_y_high{quat_y_high},
    quat_z_low{quat_z_low},
    quat_z_high{quat_z_high},
    euler_pitch{euler_pitch},
    euler_roll{euler_roll},
    euler_yaw{euler_yaw},
    temperature{temperature},
    sensors{sensors},
    cmd_led{cmd_led},
    checksum{checksum}
  {}

  SerialFeedback & operator=(const SerialFeedback & other)
  {
    if (this != &other) {
      start = other.start;
      cmd_l = other.cmd_l;
      cmd_r = other.cmd_r;
      speed_r_meas = other.speed_r_meas;
      speed_l_meas = other.speed_l_meas;
      bat_voltage = other.bat_voltage;
      board_temp = other.board_temp;
      gyro_x = other.gyro_x;
      gyro_y = other.gyro_y;
      gyro_z = other.gyro_z;
      accel_x = other.accel_x;
      accel_y = other.accel_y;
      accel_z = other.accel_z;
      quat_w_low = other.quat_w_low;
      quat_w_high = other.quat_w_high;
      quat_x_low = other.quat_x_low;
      quat_x_high = other.quat_x_high;
      quat_y_low = other.quat_y_low;
      quat_y_high = other.quat_y_high;
      quat_z_low = other.quat_z_low;
      quat_z_high = other.quat_z_high;
      euler_pitch = other.euler_pitch;
      euler_roll = other.euler_roll;
      euler_yaw = other.euler_yaw;
      temperature = other.temperature;
      sensors = other.sensors;
      cmd_led = other.cmd_led;
      checksum = other.checksum;
    }
    return *this;
  }

  SerialFeedback(SerialFeedback & other)
  : start{other.start},
    cmd_l{other.cmd_l},
    cmd_r{other.cmd_r},
    speed_r_meas{other.speed_r_meas},
    speed_l_meas{other.speed_l_meas},
    bat_voltage{other.bat_voltage},
    board_temp{other.board_temp},
    gyro_x{other.gyro_x},
    gyro_y{other.gyro_y},
    gyro_z{other.gyro_z},
    accel_x{other.accel_x},
    accel_y{other.accel_y},
    accel_z{other.accel_z},
    quat_w_low{other.quat_w_low},
    quat_w_high{other.quat_w_high},
    quat_x_low{other.quat_x_low},
    quat_x_high{other.quat_x_high},
    quat_y_low{other.quat_y_low},
    quat_y_high{other.quat_y_high},
    quat_z_low{other.quat_z_low},
    quat_z_high{other.quat_z_high},
    euler_pitch{other.euler_pitch},
    euler_roll{other.euler_roll},
    euler_yaw{other.euler_yaw},
    temperature{other.temperature},
    sensors{other.sensors},
    cmd_led{other.cmd_led},
    checksum{other.checksum}
  {}

  SerialFeedback(SerialFeedback && other)
  : start{other.start},
    cmd_l{other.cmd_l},
    cmd_r{other.cmd_r},
    speed_r_meas{other.speed_r_meas},
    speed_l_meas{other.speed_l_meas},
    bat_voltage{other.bat_voltage},
    board_temp{other.board_temp},
    gyro_x{other.gyro_x},
    gyro_y{other.gyro_y},
    gyro_z{other.gyro_z},
    accel_x{other.accel_x},
    accel_y{other.accel_y},
    accel_z{other.accel_z},
    quat_w_low{other.quat_w_low},
    quat_w_high{other.quat_w_high},
    quat_x_low{other.quat_x_low},
    quat_x_high{other.quat_x_high},
    quat_y_low{other.quat_y_low},
    quat_y_high{other.quat_y_high},
    quat_z_low{other.quat_z_low},
    quat_z_high{other.quat_z_high},
    euler_pitch{other.euler_pitch},
    euler_roll{other.euler_roll},
    euler_yaw{other.euler_yaw},
    temperature{other.temperature},
    sensors{other.sensors},
    cmd_led{other.cmd_led},
    checksum{other.checksum}
  {}
};
}  // namespace paxi_hardware

#endif  // PAXI_HARDWARE__HOVERBOARD_PROTOCOL_STRUCT_HPP_
