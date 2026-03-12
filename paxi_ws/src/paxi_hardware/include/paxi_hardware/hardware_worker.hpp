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

#ifndef PAXI_HARDWARE__HARDWARE_WORKER_HPP_
#define PAXI_HARDWARE__HARDWARE_WORKER_HPP_

#include <thread>

#include <atomic>
#include <vector>
// #include <memory>
// #include <cmath>
// #include <algorithm>
#include <cstdint>
#include <chrono>

#include "paxi_hardware/hardware_manager.hpp"

#include "paxi_common/hardware_logger_names.hpp"
#include "paxi_common/math.hpp"
#include "paxi_common/utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/hardware_info.hpp"

namespace paxi_hardware
{
/*
* Handles reading hoverboard hardware in a thread to get constant feedback data.
* Reads encoder information and updates state positions accordingly.
* Reads IMU sensor information and publishes data for sensor fusion.
*/
class HardwareWorker
{
public:
  explicit HardwareWorker(HardwareManager * hardware_manager_instance);
  ~HardwareWorker();

  HardwareWorker(const HardwareWorker &) = delete;
  HardwareWorker & operator=(const HardwareWorker &) = delete;

  HardwareWorker(HardwareWorker &&) noexcept = delete;
  HardwareWorker & operator=(HardwareWorker &&) noexcept = delete;

  void start_worker();
  void stop_worker();

private:
  // hardware manager should be stack allocated, we can use raw pointer here
  // Hardware Manger is only initilized once by hardware interface, its lifetime is tied
  // to the paxi_interface node

  HardwareManager * hardware_manager_;

  std::thread protocol_worker_thread_;
  std::atomic<bool> worker_running_;

  std::chrono::steady_clock::time_point no_data_last_time_;
  std::chrono::steady_clock::time_point disconnect_last_time_;

  std::size_t no_data_read_count_;
  std::size_t disconnect_read_count_;

  void worker_loop();

  void no_data_handler(const std::chrono::steady_clock::time_point & now);
  void disconnected_handler(const std::chrono::steady_clock::time_point & now);

  // Maximum number of no data reads before stopping worker
  static constexpr std::size_t MAX_NO_DATA_READS = 10;

  // Maximum number of read returning -1 (indicating failure/likely disconnect)
  static constexpr std::size_t MAX_DISCONNECTED_READS = 10;

  // Time within hardware_interface has to have reached maximum reads
  static constexpr std::chrono::duration<double> MAX_NO_READ_WINDOW_SEC = std::chrono::seconds(1);

  // Time within hardware_interface has to have reached maximum reads
  static constexpr std::chrono::duration<double> MAX_DISCONNECT_READ_WINDOW_SEC =
    std::chrono::seconds(1);

  // Microsecond delay before retrying a bad/failed read
  static constexpr std::size_t READ_RETRY_DELAY_MICROSEC = 500;
};
}  // namespace paxi_hardware

#endif  // PAXI_HARDWARE__HARDWARE_WORKER_HPP_
