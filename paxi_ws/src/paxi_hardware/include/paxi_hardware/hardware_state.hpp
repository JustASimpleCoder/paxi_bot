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

#ifndef PAXI_HARDWARE__HARDWARE_STATEHPP_
#define PAXI_HARDWARE__HARDWARE_STATEHPP_

#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>
#include <cstdint>

#include "paxi_hardware/encoder.hpp"
#include "paxi_hardware/hoverboard_protocol.hpp"
#include "paxi_hardware/imu.hpp"
#include "paxi_hardware/paxi_interface_node.hpp"
#include "paxi_hardware/serial_port.hpp"
#include "paxi_hardware/utility.hpp"

#include "paxi_common/hardware_logger_names.hpp"
#include "paxi_common/math.hpp"
#include "paxi_common/utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/hardware_info.hpp"

namespace paxi_hardware
{
class HardwareState
{
public:
  /*
  * Handles reading/writing to hoverboard hardware in a thread.
  * Reads enocer information and updates state positions accordingly.
  * Reads IMU sensor information and publishes data for sensor fusion.
  */
  HardwareState();
  ~HardwareState() = default;

  HardwareState(const HardwareState &) = delete;
  HardwareState & operator=(const HardwareState &) = delete;

  HardwareState(HardwareState &&) noexcept = delete;
  HardwareState & operator=(HardwareState &&) noexcept = delete;

  void init_state_interfaces(
    const hardware_interface::HardwareInfo & hardware_info,
    std::vector<double> & state_position,
    std::vector<double> & state_velocity,
    std::vector<double> & hw_commands
  );

  void activate_state_interfaces(
    std::vector<double> & state_position,
    std::vector<double> & state_velocity,
    std::vector<double> & hw_commands
  );

  bool set_hardware_params_from_xacro(const hardware_interface::HardwareInfo & hardware_info);

  //void write_command(const double l_wheel_cmd, const double r_wheel_cmd);
  void write_hover_command(const SerialCommand & hover_cmd);
  void retry_hover_command(const SerialCommand & hover_cmd);
  void publish_imu_data(const rclcpp::Time & time);
  void protocol_parsing_loop(const ssize_t bytes_read);

  SerialCommand get_cmd_from_controller(const double l_wheel_cmd, const double r_wheel_cmd);
  SerialCommand get_calibration_cmd_from_controller(
    const double l_wheel_cmd,
    const double r_wheel_cmd
  );

  ssize_t get_new_feedback_buffer();


  inline bool open_serial_port()
  {
    std::scoped_lock lock(mutex_serial_);
    return serial_port_.open_port();
  }

  inline bool is_serial_port_open() const
  {
    std::scoped_lock lock(mutex_serial_);
    return serial_port_.is_open();
  }

  inline void close_serial_port()
  {
    std::scoped_lock lock(mutex_serial_);
    serial_port_.close_port();
  }

  inline void update_serial_port_connection()
  {
    std::scoped_lock lock(mutex_serial_);
    serial_port_.update_connection();
  }

  void copy_state_interfaces(
    std::vector<double> & state_positions,
    std::vector<double> & state_velocities) const
  {
    std::scoped_lock lock(mutex_state_);
    state_positions = state_interface_positions_buf_;
    state_velocities = state_interface_velocities_buf_;
  }

private:
  // Chosen to place this on stack versus heap with smart pointers.
  // Classses are simple enough with small & mostly primitive type resources
  SerialPort serial_port_;
  HoverboardProtocol protocol_;
  EncoderKinematics encoder_kin_;
  ImuProcessing imu_;

  std::vector<double> state_interface_positions_buf_;
  std::vector<double> state_interface_velocities_buf_;

  void update_paxi_interface_state();

  // Buffer size for sample of uint_8t feedback data, 256 more than enough, each feedback stuct is
  // about ~44 bytes. To small and will miss some data, too big and larget time inbetween reads
  static constexpr std::size_t CONTROLLER_FEEDBACK_BUFFER = 256;

  std::array<std::uint8_t, CONTROLLER_FEEDBACK_BUFFER> feedback_buf_;

  mutable std::mutex mutex_state_;
  mutable std::mutex mutex_serial_;

  std::unique_ptr<PaxiInterfaceNode> paxi_interface_node_;
  rclcpp::Clock::SharedPtr cached_clock_;

  double l_constant_from_lin_reg_model(const double rpm_target);
  double r_constant_from_lin_reg_model(const double rpm_target);

  // number of times to retry writing the same command before entering failure
  static constexpr std::size_t MAX_RETRY_WRITE_COMMAND = 3;
};
}  // namespace paxi_hardware

#endif  // PAXI_HARDWARE__HARDWARE_STATEHPP_
