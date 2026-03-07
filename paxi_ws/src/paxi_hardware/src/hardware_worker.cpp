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
using paxi_common::hardware_loggers::LOGGER_PROTOCOL_WORKER;
using paxi_common::math::RPM_TO_RAD_S;
using paxi_common::math::RAD_S_TO_RPM;

using paxi_common::utils::to_index;
using paxi_common::utils::Wheel;

using std::chrono::steady_clock;

HardwareWorker::HardwareWorker(HardwareManager * hardware_manager_instance)
: hardware_manager_{hardware_manager_instance},
  protocol_worker_thread_{},
  worker_running_{false},
  no_data_last_time_{steady_clock::now()},
  disconnect_last_time_{steady_clock::now()},
  no_data_read_count_{0},
  disconnect_read_count_{0}
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

void HardwareWorker::start_worker()
{
  worker_running_ = true;
  protocol_worker_thread_ = std::thread(&HardwareWorker::worker_loop, this);
  std::thread tester = std::thread();
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
    const ssize_t bytes_read = hardware_manager_->get_new_feedback_buffer();
    steady_clock::time_point now = steady_clock::now();

    if (bytes_read > 0) {
      no_data_read_count_ = 0;
      disconnect_read_count_ = 0;
      hardware_manager_->protocol_parsing_loop(bytes_read);
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

void HardwareWorker::no_data_handler(const steady_clock::time_point & now)
{
  if (now - no_data_last_time_ <
    std::chrono::duration(MAX_NO_READ_WINDOW_SEC))
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

void HardwareWorker::disconnected_handler(const steady_clock::time_point & now)
{
  hardware_manager_->update_serial_port_connection();

  if (now - disconnect_last_time_ <
    std::chrono::duration(MAX_DISCONNECT_READ_WINDOW_SEC))
  {
    ++disconnect_read_count_;
  } else {
    disconnect_read_count_ = 0;
  }

  disconnect_last_time_ = now;

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
}  // namespace paxi_hardware
