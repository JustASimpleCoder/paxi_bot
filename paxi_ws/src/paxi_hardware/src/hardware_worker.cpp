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

namespace paxi_hardware
{
HardwareWorker::HardwareWorker()
:   serial_port_{},
  protocol_{},
  encoder_kin_{},
  imu_{},
  state_interface_positions_buf_{},
  state_interface_velocities_buf_{},
  feedback_buf_{},
  protocol_worker_thread_{},
  worker_running_{false},
  mutex_state_{},
  mutex_serial_{},
  paxi_interface_node_{std::make_unique<PaxiInterfaceNode>()},
  cached_clock_{paxi_interface_node_->get_clock()},
  no_data_read_count_{0},
  disconnect_read_count_{0},
  no_data_last_time_{cached_clock_->now()},
  disconnect_read_time_{cached_clock_->now()}
{}

void HardwareWorker::init_zero_state_interfaces(
  const hardware_interface::HardwareInfo & hardware_info,
  std::vector<double> & state_positions,
  std::vector<double> & state_velocities,
  std::vector<double> & hw_commands)
{
  const std::size_t joint_size = hardware_info.joints.size();

  auto init_vectors = [joint_size](std::vector<double> & v) ->void
    {
      v.reserve(joint_size);
      v.resize(joint_size, 0.0);
    };
  init_vectors(state_interface_positions_buf_);
  init_vectors(state_interface_velocities_buf_);

  init_vectors(state_positions);
  init_vectors(state_velocities);
  init_vectors(hw_commands);
}

bool HardwareWorker::set_hardware_params_from_xacro(
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

    validate_params &= encoder_kin_.set_wheel_radius(
      std::stod(hardware_info.hardware_parameters.at("wheel_radius"))
    );

    validate_params &= encoder_kin_.set_wheel_separation(
      std::stod(hardware_info.hardware_parameters.at("wheel_separation"))
    );

    validate_params &= encoder_kin_.set_max_velocity(
      std::stod(hardware_info.hardware_parameters.at("max_velocity"))
    );

    validate_params &= imu_.set_imu_link_name(
      hardware_info.hardware_parameters.at("imu_link_name")
    );
  } catch (const std::out_of_range & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger(LOGGER_HARDWARE),
      "Unable to parse parameters required from XACRO file:  %s",
      e.what()
    );
    return false;
  }
  return validate_params;
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
    const ssize_t bytes_read = get_new_feedback_buffer();
    rclcpp::Time now = cached_clock_->now();

    if (bytes_read > 0) {
      no_data_read_count_ = 0;
      disconnect_read_count_ = 0;
      protocol_parsing_loop(bytes_read);
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
  serial_port_.update_connection();

  if (now - disconnect_read_time_ <
    rclcpp::Duration::from_seconds(MAX_FAILURE_READ_WINDOW_SEC))
  {
    ++disconnect_read_count_;
  } else {
    disconnect_read_count_ = 0;
  }

  disconnect_read_time_ = now;

  if (disconnect_read_count_ > MAX_DISCONNECTED_READS) {
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

ssize_t HardwareWorker::get_new_feedback_buffer()
{
  std::scoped_lock<std::mutex> lock(mutex_serial_);
  return serial_port_.read_into_uint8_buf(
    feedback_buf_.data(),
    feedback_buf_.size()
  );
}

void HardwareWorker::protocol_parsing_loop(const ssize_t bytes_read)
{
  for (auto i = 0u; i < static_cast<size_t>(bytes_read); ++i) {
    if (!protocol_.process_byte(feedback_buf_[i])) {
      continue;
    }

    update_paxi_interface_state();
  }
}

const sensor_msgs::msg::Imu & HardwareWorker::update_paxi_interface_state()
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

  if constexpr (DEBUG_SENSORS) {
    paxi_interface_node_->publish_real_time(feedback, false, state_interface_positions_buf_);
  }

  return imu_.get_imu_msg();
}

void HardwareWorker::write_command(const std::vector<double> & hw_command)
{
  SerialCommand hover_cmd = get_hover_cmd_from_controller(hw_command);

  if constexpr (DEBUG_SENSORS) {
    paxi_interface_node_->publish_cmd_to_hover(hover_cmd);
  }

  write_hover_command(hover_cmd);
}

SerialCommand HardwareWorker::get_hover_cmd_from_controller(const std::vector<double> & hw_command)
{
  auto to_rpm_int16 = [] (const double val, const double conversion_const) noexcept->int16_t
  {
    const double tmp = std::round(val * conversion_const);
    // We won't worry about overflow, hoverboard wheels should not ever be spinning below -32768
    // or above 32768 especially with velocity limits from controller.yaml
    return static_cast<int16_t>(tmp);
  };

  return protocol_.to_serial_command(
    to_rpm_int16(hw_command[to_index(Wheel::LEFT)], L_RPM_CONVERSION),
    to_rpm_int16(hw_command[to_index(Wheel::RIGHT)], R_RPM_CONVERSION)
  );
}

void HardwareWorker::write_hover_command(const SerialCommand & hover_cmd)
{
  std::scoped_lock<std::mutex> lock(mutex_serial_);
  if (serial_port_.write_port(hover_cmd) < 0) {
    RCLCPP_WARN(
      rclcpp::get_logger(LOGGER_HARDWARE),
      "Protocol failed to send command to port [%s], retrying [%zu] times",
      serial_port_.get_port_name().c_str(),
      MAX_RETRY_WRITE_COMMAND
    );
    retry_hover_command(hover_cmd);
  }
}

void HardwareWorker::retry_hover_command(const SerialCommand & hover_cmd)
{
  for (size_t attempt = 0; attempt < MAX_RETRY_WRITE_COMMAND; ++attempt) {
    if (serial_port_.write_port(hover_cmd) >= 0) {
      return;
    }

    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger(LOGGER_HARDWARE),
      *cached_clock_,
      1000,
      "Failed to write hover command [%zu] time(s) for port [%s]",
      attempt,
      serial_port_.get_port_name().c_str()
    );
  }

  RCLCPP_FATAL(
    rclcpp::get_logger(LOGGER_PROTOCOL_WORKER),
    "Failed to write hover commands, stopping worker"
  );

  worker_running_ = false;
}

void HardwareWorker::publish_imu_data(const rclcpp::Time & time)
{
  std::scoped_lock<std::mutex> lock(mutex_state_);
  imu_.update_imu_msg_time(time);
  paxi_interface_node_->publish_imu_msg(
    imu_.get_imu_msg()
  );
}
}  // namespace paxi_hardware
