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

// Protocol support file
// Based on Arduino sample https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC/blob/master/Arduino/hoverserial/hoverserial.ino
// From Emanuel FERU's hoverboard-firmware-hack-FOC firmware

#ifndef PAXI_HARDWARE__HOVERBOARD_PROTOCOL_HPP_
#define PAXI_HARDWARE__HOVERBOARD_PROTOCOL_HPP_

#include <cstdint>

#include "paxi_common/hardware_logger_names.hpp"
#include "paxi_hardware/hoverboard_protocol_struct.hpp"
#include "rclcpp/rclcpp.hpp"

namespace paxi_hardware
{
class HoverboardProtocol
{
  /*
* Handles Protocol for communicating with hoverboard
*
*/

public:
  HoverboardProtocol() = default;

  HoverboardProtocol(const HoverboardProtocol &) = delete;
  HoverboardProtocol(HoverboardProtocol && other) noexcept;
  HoverboardProtocol & operator=(const HoverboardProtocol &) = delete;
  HoverboardProtocol & operator=(HoverboardProtocol &&) noexcept;

  bool process_byte(std::uint8_t incoming_byte);
  SerialCommand to_serial_command(std::int16_t l_speed, std::int16_t r_speed);

  const SerialFeedback & get_feedback() const noexcept { return feedback_; }
  const SerialCommand & get_command() const noexcept { return command_; }

private:
  SerialCommand command_;
  SerialFeedback feedback_;
  SerialFeedback new_feedback_;

  static constexpr std::size_t MAX_FEEDBACK_PACKET_SIZE = sizeof(SerialFeedback);

  std::array<std::uint8_t, MAX_FEEDBACK_PACKET_SIZE> buf_;
  std::uint16_t start_frame_;
  std::uint8_t prev_byte_;
  std::size_t msg_len_ = 0;

  // Feedback data protocol starts with 0xABCD
  static constexpr std::uint16_t START_FRAME = 0xABCD;
};

}  // namespace paxi_hardware

#endif  // PAXI_HARDWARE__HOVERBOARD_PROTOCOL_HPP_
