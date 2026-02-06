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

#include "paxi_hardware/paxi_interface.hpp"

namespace paxi_hardware
{

PaxiInterface::PaxiInterface()
:   hoverboard_worker_{}
{}

hardware_interface::return_type PaxiInterface::prepare_command_mode_switch(
  const std::vector<std::string> &, const std::vector<std::string> &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PaxiInterface::perform_command_mode_switch(
  const std::vector<std::string> &, const std::vector<std::string> &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn PaxiInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PaxiInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PaxiInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PaxiInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PaxiInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!hoverboard_worker_.open_serial_port()) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_HARDWARE), "Failed to open serial port to hoverboard");
    return hardware_interface::CallbackReturn::ERROR;
  }

  hoverboard_worker_.start_worker();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PaxiInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hoverboard_worker_.close_serial_port();
  if (hoverboard_worker_.is_serial_port_open()) {
    RCLCPP_INFO(
      rclcpp::get_logger(LOGGER_HARDWARE),
      "Failed to close port, paxi hardware still active!"
    );
  }

  RCLCPP_INFO(
    rclcpp::get_logger(LOGGER_HARDWARE),
    "Successfully closed port, hoverboard hardware deactivated!"
  );

  hoverboard_worker_.stop_worker();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PaxiInterface::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  if (hardware_interface::SystemInterface::on_init(hardware_info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!get_params_from_xacro(hardware_info)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!check_joints_and_state(hardware_info)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hoverboard_worker_.init_zero_state_interfaces(
    hardware_info,
    state_interface_positions_,
    state_interface_velocities_,
    hw_commands_
  );

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool PaxiInterface::get_params_from_xacro(const hardware_interface::HardwareInfo & hardware_info)
{
  bool validate_params = hoverboard_worker_.set_hardware_params_from_xacro(hardware_info);
  if (!validate_params) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        LOGGER_HARDWARE),
      "One or more XACRO parameters failed to set, please look at"
      "previous errors for specific paramters"
    );
  }

  return validate_params;
}

bool PaxiInterface::check_joints_and_state(const hardware_interface::HardwareInfo & hardware_info)
{
  auto log_size_error = [](const hardware_interface::ComponentInfo & joint,
      const std::string & what,
      std::size_t expected,
      std::size_t actual) {
      RCLCPP_FATAL(
        rclcpp::get_logger(LOGGER_HARDWARE),
        "Joint '%s' has %zu %s interface(s). %zu expected.",
        joint.name.c_str(),
        actual,
        what.c_str(),
        expected
      );
    };

  auto log_name_error = [](const hardware_interface::ComponentInfo & joint,
      const std::string & what,
      const std::string & expected,
      const std::string & actual) {
      RCLCPP_FATAL(
        rclcpp::get_logger(LOGGER_HARDWARE),
        "Joint '%s' has '%s' as %s interface. '%s' expected.",
        joint.name.c_str(),
        actual.c_str(),
        what.c_str(),
        expected.c_str()
      );
    };

  for (const auto & joint : hardware_info.joints) {
    if (joint.command_interfaces.size() != 1) {
      log_size_error(joint, "command", 1, joint.command_interfaces.size());
      return false;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      log_name_error(
        joint, "command", hardware_interface::HW_IF_VELOCITY,
        joint.command_interfaces[0].name);
      return false;
    }

    if (joint.state_interfaces.size() != 2) {
      log_size_error(joint, "state", 2, joint.state_interfaces.size());
      return false;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      log_name_error(
        joint, "first state", hardware_interface::HW_IF_POSITION,
        joint.state_interfaces[0].name);
      return false;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      log_name_error(
        joint, "second state", hardware_interface::HW_IF_VELOCITY,
        joint.state_interfaces[1].name);
      return false;
    }
  }
  return true;
}

std::vector<hardware_interface::StateInterface> PaxiInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size());

  for (auto i = 0u; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &state_interface_positions_[i]
      )
    );
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &state_interface_velocities_[i]
      )
    );
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> PaxiInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (auto i = 0u; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &hw_commands_[i]
      )
    );
  }

  return command_interfaces;
}
hardware_interface::return_type PaxiInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  hoverboard_worker_.publish_imu_data(time);
  hoverboard_worker_.copy_state_interfaces(
    state_interface_positions_,
    state_interface_velocities_
  );

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PaxiInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  hoverboard_worker_.write_command(hw_commands_);
  return hardware_interface::return_type::OK;
}
}  // namespace paxi_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(paxi_hardware::PaxiInterface, hardware_interface::SystemInterface)
