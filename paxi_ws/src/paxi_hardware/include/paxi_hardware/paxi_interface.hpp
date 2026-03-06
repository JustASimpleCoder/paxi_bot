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

#ifndef PAXI_HARDWARE__PAXI_INTERFACE_HPP_
#define PAXI_HARDWARE__PAXI_INTERFACE_HPP_

#include <array>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "paxi_hardware/hardware_worker.hpp"
#include "paxi_common/hardware_logger_names.hpp"

namespace paxi_hardware
{
class PaxiInterface : public hardware_interface::SystemInterface
{
public:
  PaxiInterface();

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info)
  override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> &, const std::vector<std::string> &) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> &, const std::vector<std::string> &) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Chosen to place this on stack versus heap with smart pointers.
  // Class is simple enough with small & mostly primitive type resources
  HardwareWorker hoverboard_worker_;

  std::vector<double> state_interface_positions_;
  std::vector<double> state_interface_velocities_;
  std::vector<double> hw_commands_;

  [[nodiscard]] bool get_params_from_xacro(const hardware_interface::HardwareInfo & hardware_info);
  [[nodiscard]] bool check_joints_and_state(
    const hardware_interface::HardwareInfo & hardware_info);
};

}  // namespace paxi_hardware

#endif  // PAXI_HARDWARE__PAXI_INTERFACE_HPP_
