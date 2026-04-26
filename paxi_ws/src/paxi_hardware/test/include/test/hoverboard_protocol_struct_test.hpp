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

#ifndef HOVERBOARD_PROTOCOL_STRUCT_TEST_HPP_
#define HOVERBOARD_PROTOCOL_STRUCT_TEST_HPP_

#include <cstdint>
#include <utility>
#include <vector>

#include <paxi_hardware/hoverboard_protocol_struct.hpp>
namespace paxi_hardware
{

SerialFeedback create_serial_feedback()
{
  SerialFeedback feedback;
  feedback.start = 0;
  feedback.cmd_l = 0;
  feedback.cmd_r = 0;
  feedback.speed_r_meas = 0;
  feedback.speed_l_meas = 0;
  feedback.bat_voltage = 0;
  feedback.board_temp = 0;
  feedback.gyro_x = 0;
  feedback.gyro_y = 0;
  feedback.gyro_z = 0;
  feedback.accel_x = 0;
  feedback.accel_y = 0;
  feedback.accel_z = 0;
  feedback.quat_w_low = 0;
  feedback.quat_w_high = 0;
  feedback.quat_x_low = 0;
  feedback.quat_x_high = 0;
  feedback.quat_y_low = 0;
  feedback.quat_y_high = 0;
  feedback.quat_z_low = 0;
  feedback.quat_z_high = 0;
  feedback.euler_pitch = 0;
  feedback.euler_roll = 0;
  feedback.euler_yaw = 0;
  feedback.temperature = 0;
  feedback.sensors = 0;
  feedback.cmd_led = 0;
  feedback.checksum = 0;
  return feedback;
}

inline SerialFeedback create_serial_feedback(
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
{
  SerialFeedback feedback;
  feedback.start = start,
  feedback.cmd_l = cmd_l;
  feedback.cmd_r = cmd_r;
  feedback.speed_r_meas = speed_r_meas;
  feedback.speed_l_meas = speed_l_meas;
  feedback.bat_voltage = bat_voltage;
  feedback.board_temp = board_temp;
  feedback.gyro_x = gyro_x;
  feedback.gyro_y = gyro_y;
  feedback.gyro_z = gyro_z;
  feedback.accel_x = accel_x;
  feedback.accel_y = accel_y;
  feedback.accel_z = accel_z;
  feedback.quat_w_low = quat_w_low;
  feedback.quat_w_high = quat_w_high;
  feedback.quat_x_low = quat_x_low;
  feedback.quat_x_high = quat_x_high;
  feedback.quat_y_low = quat_y_low;
  feedback.quat_y_high = quat_y_high;
  feedback.quat_z_low = quat_z_low;
  feedback.quat_z_high = quat_z_high;
  feedback.euler_pitch = euler_pitch;
  feedback.euler_roll = euler_roll;
  feedback.euler_yaw = euler_yaw;
  feedback.temperature = temperature;
  feedback.sensors = sensors;
  feedback.cmd_led = cmd_led;
  feedback.checksum = checksum;
  return feedback;
}

inline SerialFeedback create_serial_feedback(const SerialFeedback & other_feedback)
{
  SerialFeedback feedback;
  feedback.start = other_feedback.start,
  feedback.cmd_l = other_feedback.cmd_l;
  feedback.cmd_r = other_feedback.cmd_r;
  feedback.speed_r_meas = other_feedback.speed_r_meas;
  feedback.speed_l_meas = other_feedback.speed_l_meas;
  feedback.bat_voltage = other_feedback.bat_voltage;
  feedback.board_temp = other_feedback.board_temp;
  feedback.gyro_x = other_feedback.gyro_x;
  feedback.gyro_y = other_feedback.gyro_y;
  feedback.gyro_z = other_feedback.gyro_z;
  feedback.accel_x = other_feedback.accel_x;
  feedback.accel_y = other_feedback.accel_y;
  feedback.accel_z = other_feedback.accel_z;
  feedback.quat_w_low = other_feedback.quat_w_low;
  feedback.quat_w_high = other_feedback.quat_w_high;
  feedback.quat_x_low = other_feedback.quat_x_low;
  feedback.quat_x_high = other_feedback.quat_x_high;
  feedback.quat_y_low = other_feedback.quat_y_low;
  feedback.quat_y_high = other_feedback.quat_y_high;
  feedback.quat_z_low = other_feedback.quat_z_low;
  feedback.quat_z_high = other_feedback.quat_z_high;
  feedback.euler_pitch = other_feedback.euler_pitch;
  feedback.euler_roll = other_feedback.euler_roll;
  feedback.euler_yaw = other_feedback.euler_yaw;
  feedback.temperature = other_feedback.temperature;
  feedback.sensors = other_feedback.sensors;
  feedback.cmd_led = other_feedback.cmd_led;
  feedback.checksum = other_feedback.checksum;
  return feedback;
}

struct TestRealTimeParams
{
  SerialFeedback feedback;
  bool connected;
  std::vector<double> state_positions;

  TestRealTimeParams()
  {
    feedback = create_serial_feedback();
    connected = false;
    state_positions = {};
  }

  TestRealTimeParams(
    SerialFeedback other_feedback, bool other_connected,
    std::vector<double> other_state_positions)
  : connected{other_connected}, state_positions{other_state_positions}
  {
    feedback = create_serial_feedback(other_feedback);
  }

  TestRealTimeParams(const TestRealTimeParams & other)
  {
    feedback = other.feedback;
    connected = other.connected;
    state_positions = other.state_positions;
  }

  TestRealTimeParams(const TestRealTimeParams && other)
  {
    feedback = other.feedback;
    connected = other.connected;
    state_positions = other.state_positions;
  }


  TestRealTimeParams & operator=(const TestRealTimeParams & other)
  {
    if (this != &other) {
      feedback = other.feedback;
      connected = other.connected;
      state_positions = other.state_positions;
    }
    return *this;
  }

  TestRealTimeParams & operator=(TestRealTimeParams && other)
  {
    if (this != &other) {
      feedback = create_serial_feedback(other.feedback);
      connected = other.connected;
      state_positions = std::move(other.state_positions);
    }
    return *this;
  }
};


paxi_msgs::msg::ControllerCommand create_controller_cmd()
{
  paxi_msgs::msg::ControllerCommand cmd;
  cmd.l_speed = 0;
  cmd.r_speed = 0;
  return cmd;
}

paxi_msgs::msg::ControllerCommand create_controller_cmd(double l_speed, double r_speed)
{
  paxi_msgs::msg::ControllerCommand cmd;
  cmd.l_speed = l_speed;
  cmd.r_speed = r_speed;
  return cmd;
}


}  // namespace paxi_hardware

#endif  // HOVERBOARD_PROTOCOL_STRUCT_TEST_HPP_
