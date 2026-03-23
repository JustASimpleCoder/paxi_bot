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

#ifndef PAXI_HARDWARE__SERIAL_PORT_HPP_
#define PAXI_HARDWARE__SERIAL_PORT_HPP_

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

#include "paxi_common/hardware_logger_names.hpp"
#include "paxi_hardware/hoverboard_protocol_struct.hpp"
#include "rclcpp/rclcpp.hpp"

namespace paxi_hardware
{
/*
  * Thin c++ wrapper around C termios library for dealing with initializing, writing and reading
  * POSIX port data. Only works on Linux systems
  */
class SerialPort
{
public:
  SerialPort();
  SerialPort(const std::string & port, std::uint32_t baud_rate);
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  SerialPort(SerialPort && other) noexcept;
  SerialPort & operator=(SerialPort && other) noexcept;

  [[nodiscard]] bool open_port();
  void close_port();

  ssize_t write_port(const SerialCommand & cmd) const;

  ssize_t read_into_uint8_buf(std::uint8_t * buffer, std::size_t max_len) const;

  [[nodiscard]] bool set_port(const std::string & port_name);
  [[nodiscard]] bool set_baud(std::uint32_t baud_rate);

  void update_connection();

  // inline void flush_port() {tcflush(fd_, TCIFLUSH);}
  inline std::string get_port_name() const noexcept { return port_; }
  inline std::uint32_t get_baud() const noexcept { return baud_rate_; }
  inline int get_port_fd() const noexcept { return fd_; }

  inline bool is_open() const noexcept { return fd_ != -1; }
  inline bool is_connected() const noexcept { return connected_; }

private:
  std::string port_;
  std::uint32_t baud_rate_;
  int fd_;
  bool connected_;
};
}  // namespace paxi_hardware

#endif  // PAXI_HARDWARE__SERIAL_PORT_HPP_
