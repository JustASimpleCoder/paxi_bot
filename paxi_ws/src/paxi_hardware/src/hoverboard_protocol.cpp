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

#include "paxi_hardware/hoverboard_protocol.hpp"

#include "rclcpp/rclcpp.hpp"
namespace paxi_hardware
{
HoverboardProtocol::HoverboardProtocol(HoverboardProtocol && other) noexcept
:   command_(std::move(other.command_)),
  feedback_(std::move(other.feedback_)),
  new_feedback_(std::move(other.new_feedback_)),
  buf_{std::move(other.buf_)},

  start_frame_(other.start_frame_),
  prev_byte_(other.prev_byte_),
  msg_len_(other.msg_len_)
{}

HoverboardProtocol & HoverboardProtocol::operator=(HoverboardProtocol && other) noexcept
{
  if (this != &other) {
    command_ = std::move(other.command_);
    feedback_ = std::move(other.feedback_);
    new_feedback_ = std::move(other.new_feedback_);
    buf_ = std::move(other.buf_);

    start_frame_ = other.start_frame_;
    prev_byte_ = other.prev_byte_;
    msg_len_ = other.msg_len_;
  }

  return *this;
}

SerialCommand HoverboardProtocol::to_serial_command(int16_t l_speed, int16_t r_speed)
{
  command_.start = static_cast<uint16_t>(K_START_FRAME);
  command_.l_speed = static_cast<int16_t>(l_speed);
  command_.r_speed = static_cast<int16_t>(r_speed);
  command_.checksum = static_cast<uint16_t>(command_.start ^ command_.l_speed ^ command_.r_speed);
  return command_;
}

bool HoverboardProtocol::process_byte(uint8_t incoming_byte)
{
  start_frame_ = (static_cast<uint16_t>(incoming_byte) << 8) | prev_byte_;
  if (start_frame_ == K_START_FRAME) {
    buf_[0] = prev_byte_;
    buf_[1] = incoming_byte;
    msg_len_ = 2;
  } else if (msg_len_ >= 2 && msg_len_ < MAX_FEEDBACK_PACKET_SIZE && msg_len_ < buf_.size()) {
    buf_[msg_len_] = incoming_byte;
    msg_len_++;
  }

  prev_byte_ = incoming_byte;
  if (msg_len_ != MAX_FEEDBACK_PACKET_SIZE) {
    if (msg_len_ > MAX_FEEDBACK_PACKET_SIZE) {
      msg_len_ = 0;
    }

    return false;
  }

  std::memcpy(&new_feedback_, &buf_, MAX_FEEDBACK_PACKET_SIZE);

  const uint16_t checksum = static_cast<uint16_t>(
    new_feedback_.start ^
    new_feedback_.cmd_l ^
    new_feedback_.cmd_r ^
    new_feedback_.speed_r_meas ^
    new_feedback_.speed_l_meas ^
    new_feedback_.bat_voltage ^
    new_feedback_.board_temp ^
    new_feedback_.gyro_x ^
    new_feedback_.gyro_y ^
    new_feedback_.gyro_z ^
    new_feedback_.accel_x ^
    new_feedback_.accel_y ^
    new_feedback_.accel_z ^
    new_feedback_.quat_w_low ^
    new_feedback_.quat_x_low ^
    new_feedback_.quat_y_low ^
    new_feedback_.quat_z_low ^
    new_feedback_.quat_w_high ^
    new_feedback_.quat_x_high ^
    new_feedback_.quat_y_high ^
    new_feedback_.quat_z_high ^
    new_feedback_.euler_pitch ^
    new_feedback_.euler_roll ^
    new_feedback_.euler_yaw ^
    new_feedback_.temperature ^
    new_feedback_.sensors ^
    new_feedback_.cmd_led
  );

  msg_len_ = 0;

  if (new_feedback_.start == K_START_FRAME && new_feedback_.checksum == checksum) {
    feedback_ = new_feedback_;
    return true;
  }

  return false;
}
}  // namespace paxi_hardware
