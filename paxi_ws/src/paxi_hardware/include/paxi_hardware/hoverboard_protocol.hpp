// Copyright 2025 Jacob Cohen

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Protocol support file
// Based on Arduino sample https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC/blob/master/Arduino/hoverserial/hoverserial.ino
// From Emanuel FERU's hoverboard-firmware-hack-FOC firmware

#ifndef PAXI_HARDWARE__HOVERBOARD_PROTOCOL_HPP_
#define PAXI_HARDWARE__HOVERBOARD_PROTOCOL_HPP_

#include "paxi_hardware/hoverboard_protocol_struct.hpp"
#include "paxi_hardware/utility.hpp"

#include "rclcpp/rclcpp.hpp"

namespace paxi_hardware
{

static constexpr std::size_t MAX_COMMAND_PACKET_SIZE = sizeof(SerialCommand);
static constexpr std::size_t MAX_FEEDBACK_PACKET_SIZE = sizeof(SerialFeedback);

class HoverboardProtocol
{
public:
  HoverboardProtocol() = default;

  HoverboardProtocol(const HoverboardProtocol &) = delete;
  HoverboardProtocol(HoverboardProtocol && other) noexcept;
  HoverboardProtocol & operator=(const HoverboardProtocol &) = delete;
  HoverboardProtocol & operator=(HoverboardProtocol &&) noexcept;

  bool process_byte(uint8_t incoming_byte);
  SerialCommand to_serial_command(int16_t l_speed, int16_t r_speed);

  const SerialFeedback & get_feedback() const noexcept {return feedback_;}
  const SerialCommand & get_command() const noexcept {return command_;}

private:
  SerialCommand command_;
  SerialFeedback feedback_;
  SerialFeedback new_feedback_;

  std::array<uint8_t, MAX_FEEDBACK_PACKET_SIZE> buf_;
  uint16_t start_frame_;
  uint8_t prev_byte_;
  std::size_t msg_len_ = 0;
};

}  // namespace paxi_hardware

#endif  // PAXI_HARDWARE__HOVERBOARD_PROTOCOL_HPP_
