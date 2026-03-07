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

#include "paxi_hardware/hardware_worker.hpp"

// TODO(JACOB): break class up into structures
//  - maybe turn failure handler into HardwareFsm class
//  - maybe turn serialport/protocol/encoder_kin/imu into HardwareState class
namespace paxi_hardware
{
using paxi_common::hardware_loggers::LOGGER_PROTOCOL_WORKER;
using paxi_common::math::RPM_TO_RAD_S;
using paxi_common::math::RAD_S_TO_RPM;

using paxi_common::utils::to_index;
using paxi_common::utils::Wheel;

HardwareWorker::HardwareWorker()
: paxi_state_{},
  protocol_worker_thread_{},
  worker_running_{false},
  paxi_interface_node_{std::make_unique<PaxiInterfaceNode>()},
  cached_clock_{paxi_interface_node_->get_clock()},
  no_data_read_count_{0},
  disconnect_read_count_{0},
  no_data_last_time_{cached_clock_->now()},
  disconnect_read_time_{cached_clock_->now()}
{}

HardwareWorker::~HardwareWorker()
{
  if (worker_running_) {
    worker_running_ = false;
  }

  if (protocol_worker_thread_.joinable()) {
    protocol_worker_thread_.join();
    RCLCPP_INFO(
      rclcpp::get_logger(LOGGER_PROTOCOL_WORKER),
      "Stopped protocol worker thread, no longer processing feedback data!"
    );
  }
}

void HardwareWorker::init_state_interfaces(
  const hardware_interface::HardwareInfo & hardware_info,
  std::vector<double> & state_positions,
  std::vector<double> & state_velocities,
  std::vector<double> & hw_commands)
{
  paxi_state_.init_state_interfaces(hardware_info, state_positions, state_velocities, hw_commands);
}

void HardwareWorker::activate_state_interfaces(
  std::vector<double> & state_positions,
  std::vector<double> & state_velocities,
  std::vector<double> & hw_commands)
{
  paxi_state_.activate_state_interfaces(state_positions, state_velocities, hw_commands);
}

bool HardwareWorker::set_hardware_params_from_xacro(
  const hardware_interface::HardwareInfo & hardware_info)
{
  return paxi_state_.set_hardware_params_from_xacro(hardware_info);
}

void HardwareWorker::start_worker()
{
  worker_running_ = true;
  protocol_worker_thread_ = std::thread(&HardwareWorker::worker_loop, this);
  RCLCPP_INFO(
    rclcpp::get_logger(LOGGER_PROTOCOL_WORKER),
    "Starting thread for protocol worker!"
  );
}

void HardwareWorker::stop_worker()
{
  worker_running_ = false;
  if (protocol_worker_thread_.joinable()) {
    protocol_worker_thread_.join();
    RCLCPP_INFO(
      rclcpp::get_logger(LOGGER_PROTOCOL_WORKER),
      "Stopped protocol worker thread, no longer processing feedback data!"
    );
  }
}

void HardwareWorker::worker_loop()
{
  while (worker_running_) {
    const ssize_t bytes_read = paxi_state_.get_new_feedback_buffer();
    rclcpp::Time now = cached_clock_->now();

    if (bytes_read > 0) {
      no_data_read_count_ = 0;
      disconnect_read_count_ = 0;
      paxi_state_.protocol_parsing_loop(bytes_read);
      continue;
    }

    if (bytes_read == 0) {
      no_data_handler(now);
      continue;
    }

    if (bytes_read < 0) {
      disconnected_handler(now);
      continue;
    }
  }
}

void HardwareWorker::no_data_handler(const rclcpp::Time & now)
{
  if (now - no_data_last_time_ <
    rclcpp::Duration::from_seconds(MAX_FAILURE_READ_WINDOW_SEC))
  {
    ++no_data_read_count_;
  } else {
    no_data_read_count_ = 0;
  }

  no_data_last_time_ = now;

  if (no_data_read_count_ > MAX_NO_DATA_READS) {
    RCLCPP_FATAL(
      rclcpp::get_logger(LOGGER_PROTOCOL_WORKER),
      "Stopped worker because reached maximum no data reads"
    );
    worker_running_ = false;
  } else {
    // try again after half a milisecond to see if was hardware issue, reduce CPU usage on bad reads
    std::this_thread::sleep_for(
      std::chrono::microseconds(READ_RETRY_DELAY_MICROSEC));
  }
}

void HardwareWorker::disconnected_handler(const rclcpp::Time & now)
{
  // TODO(jacob): serial port polls port and changes connected member variable connected_ = false
  // maybe add this to handler somehow?
  paxi_state_.update_serial_port_connection();

  if (now - disconnect_read_time_ <
    rclcpp::Duration::from_seconds(MAX_FAILURE_READ_WINDOW_SEC))
  {
    ++disconnect_read_count_;
  } else {
    disconnect_read_count_ = 0;
  }

  disconnect_read_time_ = now;


  if (disconnect_read_count_ > MAX_DISCONNECTED_READS) {
    // TODO(jacob): maybe try to close and reopen port?
    RCLCPP_FATAL(
      rclcpp::get_logger(LOGGER_PROTOCOL_WORKER),
      "Stopped worker because USB is disconnected"
    );
    worker_running_ = false;
  } else {
    // try again after 0.5 milisecs to see if disconnected was temporary.
    // reduce CPU usage on bad reads
    std::this_thread::sleep_for(
      std::chrono::microseconds(READ_RETRY_DELAY_MICROSEC)
    );
  }
}

void HardwareWorker::write_command(const double l_wheel_cmd, const double r_wheel_cmd)
{
  SerialCommand hover_cmd = paxi_state_.get_cmd_from_controller(l_wheel_cmd, r_wheel_cmd);

  if constexpr (CALIBRATE_FIRMWARE) {
    hover_cmd = paxi_state_.get_calibration_cmd_from_controller(l_wheel_cmd, r_wheel_cmd);
    paxi_interface_node_->publish_controller_cmd(l_wheel_cmd, r_wheel_cmd);
  }

  if constexpr (DEBUG_SENSORS) {
    paxi_interface_node_->publish_cmd_to_hover(hover_cmd);
  }

  paxi_state_.write_hover_command(hover_cmd);
}

}  // namespace paxi_hardware
