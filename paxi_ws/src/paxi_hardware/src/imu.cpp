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

#include "paxi_hardware/imu.hpp"

namespace paxi_hardware
{

using paxi_common::hardware_loggers::LOGGER_IMU;
using paxi_common::math::DEG_TO_RAD;

ImuProcessing::ImuProcessing()
{
  imu_msg_.header.frame_id = imu_link_name_;

  imu_msg_.orientation_covariance = {
    0.1, 0, 0,
    0, 0.1, 0,
    0, 0, 0.1,
  };

  imu_msg_.angular_velocity_covariance = {
    0.1, 0, 0,
    0, 0.1, 0,
    0, 0, 0.1,
  };

  imu_msg_.linear_acceleration_covariance = {
    0.1, 0, 0,
    0, 0.1, 0,
    0, 0, 0.1,
  };
}

bool ImuProcessing::set_imu_link_name(const std::string & link_name)
{
  if (link_name == "") {
    return false;
  }
  imu_link_name_ = link_name;
  imu_msg_.header.frame_id = imu_link_name_;
  return true;
}

void ImuProcessing::update_imu_msg_time(const rclcpp::Time & time)
{
  imu_msg_.header.stamp = time;
}

void ImuProcessing::update_imu_msg_data(const SerialFeedback & feedback)
{
  if (is_all_zero_imu_data(feedback)) {
    // super unlikely imu data will actually be all zero (always some accel)
    // used to indicate a bad data read/smooths imu data overall
    return;
  }

  // Fixed point conversion scaling factor 2^30 (float to 32 bit signed int)
  static constexpr double Q30 = 1073741824.0;

  // Converts raw acceleration data (m/s^2) from the MPU6050 to approriate gravity units 
  // (16,384 LSB/g)
  static constexpr double ACCEL_TO_G = 16384.00;

  // Converts raw gyro data (DPS units) from the MPU6050 to degree per second (16.4 LSB/(degree/s))
  static constexpr double GYRO_TO_DEG_S = 16.4;

  // Standard gravity constant 9.81 m/s^2
  static constexpr double STD_GRAVITY = 9.81;


  auto recover_quat_32_bit = [](std::int16_t high, std::uint16_t low) -> std::int32_t {
    // IMU's sideboard computes quaternions as as floats between [-1,1]. Using fixed-point 
    // conversion, with scaling factor Q30 = 2^30, qauternions are represented as 32 bits. The 
    // communication feedback protocol requires all data in the feedback structure ot be same size
    // so we send as low/ high bit, where the low bit is a unsigned integer to maintain data and 
    // not accidently interpret first bit 1 as a negative
      return (static_cast<std::int32_t>(high) << 16) | static_cast<std::int32_t>(low);
    };

  double q_w =
    static_cast<double>(recover_quat_32_bit(feedback.quat_w_high, feedback.quat_w_low)) / Q30;
  double q_x =
    static_cast<double>(recover_quat_32_bit(feedback.quat_x_high, feedback.quat_x_low)) / Q30;
  double q_y =
    static_cast<double>(recover_quat_32_bit(feedback.quat_y_high, feedback.quat_y_low)) / Q30;
  double q_z =
    static_cast<double>(recover_quat_32_bit(feedback.quat_z_high, feedback.quat_z_low)) / Q30;

  imu_msg_.angular_velocity.x = static_cast<double>(feedback.gyro_x) / GYRO_TO_DEG_S *
    DEG_TO_RAD;
  imu_msg_.angular_velocity.y = static_cast<double>(feedback.gyro_y) / GYRO_TO_DEG_S *
    DEG_TO_RAD;
  imu_msg_.angular_velocity.z = static_cast<double>(feedback.gyro_z) / GYRO_TO_DEG_S *
    DEG_TO_RAD;
    
  imu_msg_.linear_acceleration.x = static_cast<double>(feedback.accel_x) / ACCEL_TO_G *
    STD_GRAVITY;
  imu_msg_.linear_acceleration.y = static_cast<double>(feedback.accel_y) / ACCEL_TO_G *
    STD_GRAVITY;
  imu_msg_.linear_acceleration.z = static_cast<double>(feedback.accel_z) / ACCEL_TO_G *
    STD_GRAVITY;

  imu_msg_.orientation.w = q_w;
  imu_msg_.orientation.x = q_x;
  imu_msg_.orientation.y = q_y;
  imu_msg_.orientation.z = q_z;
}
}  // namespace paxi_hardware
