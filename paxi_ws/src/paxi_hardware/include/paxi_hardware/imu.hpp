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

#ifndef PAXI_HARDWARE__IMU_HPP_
#define PAXI_HARDWARE__IMU_HPP_

#include <string>
#include <cstdint>

#include "paxi_hardware/hoverboard_protocol_struct.hpp"
#include "paxi_common/hardware_logger_names.hpp"
#include "paxi_common/math.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace paxi_hardware
{
/*
* Processes IMU data from feedback from hoverboard
*/
class ImuProcessing
{
public:
  ImuProcessing();
  ~ImuProcessing() = default;

  ImuProcessing(const ImuProcessing &) = delete;
  ImuProcessing & operator=(const ImuProcessing &) = delete;

  ImuProcessing(ImuProcessing &&) noexcept = default;
  ImuProcessing & operator=(ImuProcessing &&) noexcept = default;

  [[nodiscard]] bool set_imu_link_name(const std::string & link_name);

  void update_imu_msg_time(const rclcpp::Time & time);
  void update_imu_msg_data(const SerialFeedback & feedback);

  inline const sensor_msgs::msg::Imu & get_imu_msg() const noexcept {return imu_msg_;}
  inline bool is_all_zero_imu_data(const SerialFeedback & feedback) const
  {
    // Bitwise 'OR' operation on IMU feedback data will result in zero if all bits are zeroed bits.
    // Negating result gives us true if its all zero, false otherwise.
    return !(
      feedback.gyro_x | feedback.gyro_y | feedback.gyro_z |
      feedback.accel_x | feedback.accel_y | feedback.accel_z |
      feedback.quat_w_low | feedback.quat_x_low | feedback.quat_y_low | feedback.quat_z_low |
      feedback.quat_w_high | feedback.quat_x_high | feedback.quat_y_high | feedback.quat_z_high
    );
  }

private:
  sensor_msgs::msg::Imu imu_msg_;
  std::string imu_link_name_;
};
}  // namespace paxi_hardware

#endif  // PAXI_HARDWARE__IMU_HPP_
