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

#include "paxi_hardware/hardware_manager.hpp"

namespace paxi_hardware
{
using paxi_common::hardware_loggers::LOGGER_HARDWARE_MANAGER;
using paxi_common::math::RPM_TO_RAD_S;
using paxi_common::math::RAD_S_TO_RPM;

using paxi_common::utils::to_index;
using paxi_common::utils::Wheel;

HardwareManager::HardwareManager()
:   serial_port_{},
  protocol_{},
  encoder_kin_{},
  imu_{},
  state_interface_positions_buf_{},
  state_interface_velocities_buf_{},
  feedback_buf_{},
  mutex_state_{},
  mutex_serial_{},
  paxi_interface_node_{std::make_shared<PaxiInterfaceNode>()},
  cached_clock_{paxi_interface_node_->get_clock()}
{}

void HardwareManager::init_state_interfaces(
  const hardware_interface::HardwareInfo & hardware_info,
  std::vector<double> & state_positions,
  std::vector<double> & state_velocities,
  std::vector<double> & hw_commands)
{
  const std::size_t joint_size = hardware_info.joints.size();

  auto init_vectors = [&joint_size](std::vector<double> & v) ->void
    {
      v.reserve(joint_size);
      v.resize(joint_size, std::numeric_limits<double>::quiet_NaN());
    };

  init_vectors(state_interface_positions_buf_);
  init_vectors(state_interface_velocities_buf_);

  init_vectors(state_positions);
  init_vectors(state_velocities);
  init_vectors(hw_commands);
}

void HardwareManager::activate_state_interfaces(
  std::vector<double> & state_positions,
  std::vector<double> & state_velocities,
  std::vector<double> & hw_commands)
{
  auto set_zero_vector = [](std::vector<double> & v) ->void
    {
      for (auto i = 0u; i < v.size(); ++i) {
        if (std::isnan(v[i])) {
          v[i] = 0.0;
        }
      }
    };

  set_zero_vector(state_interface_positions_buf_);
  set_zero_vector(state_interface_velocities_buf_);

  set_zero_vector(state_positions);
  set_zero_vector(state_velocities);
  set_zero_vector(hw_commands);
}

bool HardwareManager::set_hardware_params_from_xacro(
  const hardware_interface::HardwareInfo & hardware_info)
{
  bool validate_params = true;
  // .at() can throw std:: error -> indicates mismatch xacro file and lookup name,
  // use try catch to control this
  try {
    validate_params &= serial_port_.set_port(
      hardware_info.hardware_parameters.at("serial_port")
    );

    validate_params &= serial_port_.set_baud(
      std::stoul(hardware_info.hardware_parameters.at("baud_rate"))
    );

    validate_params &= imu_.set_imu_link_name(
      hardware_info.hardware_parameters.at("imu_link_name")
    );
  } catch (const std::out_of_range & e) {
    // unordered map .at() can throw out of range if no key exists
    RCLCPP_ERROR(
      rclcpp::get_logger(LOGGER_HARDWARE_MANAGER),
      "Required parameter is out of range [%s]",
      e.what()
    );
    return false;
  } catch (const std::invalid_argument & e) {
    // std::stoul can throw invalid argument if it can't convert the param
    RCLCPP_ERROR(
      rclcpp::get_logger(LOGGER_HARDWARE_MANAGER),
      "Required parameter is invalid in XACRO file:  %s",
      e.what()
    );
    return false;
  }

  return validate_params;
}

ssize_t HardwareManager::get_new_feedback_buffer()
{
  std::scoped_lock<std::mutex> lock(mutex_serial_);
  return serial_port_.read_into_uint8_buf(
    feedback_buf_.data(),
    feedback_buf_.size()
  );
}

void HardwareManager::protocol_parsing_loop(const ssize_t bytes_read)
{
  for (auto i = 0u; i < static_cast<size_t>(bytes_read); ++i) {
    if (!protocol_.process_byte(feedback_buf_[i])) {
      continue;
    }
    update_hardware_from_new_feedback();
  }
}

void HardwareManager::update_hardware_from_new_feedback()
{
  std::scoped_lock<std::mutex> lock(mutex_state_);
  const SerialFeedback & feedback = protocol_.get_feedback();
  rclcpp::Time current_time = cached_clock_->now();

  state_interface_velocities_buf_.at(to_index(Wheel::LEFT)) = feedback.speed_l_meas * RPM_TO_RAD_S;
  state_interface_velocities_buf_.at(to_index(Wheel::RIGHT)) = feedback.speed_r_meas * RPM_TO_RAD_S;

  encoder_kin_.update_angular_position(
    current_time,
    feedback.speed_r_meas,
    feedback.speed_l_meas,
    state_interface_positions_buf_
  );

  imu_.update_imu_msg_data(feedback);

  paxi_interface_node_->publish_imu_msg(imu_.get_imu_msg());
  paxi_interface_node_->publish_real_time(feedback, false, state_interface_positions_buf_);

  if constexpr (CALIBRATE_FIRMWARE) {
    paxi_interface_node_->publish_feedback(feedback);
  }
}

SerialCommand HardwareManager::get_cmd_from_controller(
  const double l_wheel_cmd,
  const double r_wheel_cmd)
{
  auto to_rpm_int16 = [] (const double rpm) noexcept->std::int16_t
  {
    // We won't worry about overflow, hoverboard wheels should not ever be spinning below -32768
    // or above 32768 especially with velocity limits from controller.yaml
    // clamp or branching can slow down a high frequency call
    const double tmp = std::round(rpm);
    return static_cast<std::int16_t>(tmp);
  };

  return protocol_.to_serial_command(
    to_rpm_int16(l_constant_from_lin_reg_model(l_wheel_cmd * RAD_S_TO_RPM)),
    to_rpm_int16(r_constant_from_lin_reg_model(r_wheel_cmd * RAD_S_TO_RPM))
  );
}

SerialCommand HardwareManager::get_calibration_cmd_from_controller(
  const double l_wheel_cmd,
  const double r_wheel_cmd)
{
  auto to_rpm_int16_clamped =
    [] (const double val, const double conversion_const) noexcept->std::int16_t
  {
    // We worry about overflow during calibration as we can get pretty high absolute values
    double tmp = std::clamp(
      std::round(val * conversion_const),
      static_cast<double>(INT16_MIN),
      static_cast<double>(INT16_MAX)
    );
    return static_cast<std::int16_t>(tmp);
  };

  // constant for calibration should be 1.0 to help find all the RPM_conversions
  return protocol_.to_serial_command(
    to_rpm_int16_clamped(l_wheel_cmd, RAD_S_TO_RPM),
    to_rpm_int16_clamped(r_wheel_cmd, RAD_S_TO_RPM)
  );
}

void HardwareManager::write_hover_command(const SerialCommand & hover_cmd)
{
  std::scoped_lock<std::mutex> lock(mutex_serial_);
  if (serial_port_.write_port(hover_cmd) < 0) {
    RCLCPP_WARN(
      rclcpp::get_logger(LOGGER_HARDWARE_MANAGER),
      "Protocol failed to send command to port [%s], retrying [%zu] times",
      serial_port_.get_port_name().c_str(),
      MAX_RETRY_WRITE_COMMAND
    );
    retry_hover_command(hover_cmd);
  }
}

void HardwareManager::retry_hover_command(const SerialCommand & hover_cmd)
{
  for (size_t attempt = 0; attempt < MAX_RETRY_WRITE_COMMAND; ++attempt) {
    if (serial_port_.write_port(hover_cmd) >= 0) {
      return;
    }
    if constexpr (DEBUG_SENSORS) {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger(LOGGER_HARDWARE_MANAGER),
        *cached_clock_,
        1000,
        "Failed to write hover command [%zu] time(s) for port [%s]",
        attempt,
        serial_port_.get_port_name().c_str()
      );
    }
  }

  RCLCPP_FATAL(
    rclcpp::get_logger(LOGGER_HARDWARE_MANAGER),
    "Failed to write hover commands, stopping worker"
  );
}


void HardwareManager::write_command(const double l_wheel_cmd, const double r_wheel_cmd)
{
  SerialCommand hover_cmd = get_cmd_from_controller(l_wheel_cmd, r_wheel_cmd);

  if constexpr (CALIBRATE_FIRMWARE) {
    hover_cmd = get_calibration_cmd_from_controller(l_wheel_cmd, r_wheel_cmd);
    paxi_interface_node_->publish_controller_cmd(l_wheel_cmd, r_wheel_cmd);
  }

  if constexpr (DEBUG_SENSORS) {
    paxi_interface_node_->publish_cmd_to_hover(hover_cmd);
  }

  write_hover_command(hover_cmd);
}


//  Values recieved from doing linear regression model
//  using the paxi_calibrate package.

//  data analysis:
//  **** LEFT WHEEL DATA POS ****
//  R-squared Left Pos [0.9998195592973994]
//  Intercept Left Pos [40.43878357330695]
//  Slope Left Pos     [[2.02618121]]

//  **** LEFT WHEEL DATA NEG ****
//  R-squared Left Pos [0.9998546047544311]
//  Intercept Left Pos [-38.450275676263146]
//  Slope Left Pos     [[2.02397546]]
double HardwareManager::l_constant_from_lin_reg_model(const double rpm_target)
{
  static constexpr double L_POS_SLOPE = 2.02618121;
  static constexpr double L_NEG_SLOPE = 2.02397546;
  static constexpr double L_POS_INTERCEPT = 40.43878;
  static constexpr double L_NEG_INTERCEPT = -38.45028;

  // f(x) = Slope*rpm + intercept
  if (rpm_target > 0) {
    return (L_POS_SLOPE * rpm_target) + L_POS_INTERCEPT;
  }
  if (rpm_target < 0) {
    return (L_NEG_SLOPE * rpm_target) + L_NEG_INTERCEPT;
  }

  return rpm_target;
}

// **** Right WHEEL DATA POS****
// R-squared Right Pos [0.9996574935016734]
// Intercept Right Pos [42.897116322354975]
// Slope Right Pos     [[2.02091579]]

// **** Right WHEEL DATA NEG****
// R-squared Right Pos [0.9996263861487986]
// Intercept Right Pos [-45.83463084595394]
// Slope Right Pos     [[2.01675397]]
double HardwareManager::r_constant_from_lin_reg_model(const double rpm_target)
{
  static constexpr double R_POS_SLOPE = 2.02091579;
  static constexpr double R_NEG_SLOPE = 2.01675397;
  static constexpr double R_POS_INTERCEPT = 42.89712;
  static constexpr double R_NEG_INTERCEPT = -45.83463;

  // f(x) = Slope*rpm + intercept
  if (rpm_target > 0) {
    return (R_POS_SLOPE * rpm_target) + R_POS_INTERCEPT;
  }
  if (rpm_target < 0) {
    return (R_NEG_SLOPE * rpm_target) + R_NEG_INTERCEPT;
  }
  return rpm_target;
}

}  // namespace paxi_hardware
