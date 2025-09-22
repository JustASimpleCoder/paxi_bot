// Protocol support file
// Based on Arduino sample https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC/blob/master/Arduino/hoverserial/hoverserial.ino
// From Emanuel FERU's hoverboard-firmware-hack-FOC firmware

#ifndef HOVERBOARD_PROTOCOL_HPP
#define HOVERBOARD_PROTOCOL_HPP

#include "paxi_hardware/hoverboard_protocol_struct.hpp"
#include "rclcpp/rclcpp.hpp"

namespace paxi_hardware
{

inline constexpr const char * LOGGER_PROTOCOL = "paxi_hardware_protocol";

static constexpr uint16_t K_START_FRAME = 0xABCD;
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
  SerialCommand to_serial_command(const int16_t & steer, const int16_t & speed);

  const SerialFeedback & get_feedback() const noexcept { return feedback_; }
  const SerialCommand & get_command() const noexcept { return command_; }

private:
  SerialCommand command_;
  SerialFeedback feedback_;
  SerialFeedback new_feedback_;

  std::array<uint8_t, MAX_FEEDBACK_PACKET_SIZE> buf_;
  uint16_t start_frame_;
  uint8_t prev_byte_;
  std::size_t msg_len_ = 0;
};

}  // end of namespace paxi_hardware
#endif
