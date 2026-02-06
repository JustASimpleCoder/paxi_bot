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

#ifndef PAXI_HARDWARE__UTILITY_HPP_
#define PAXI_HARDWARE__UTILITY_HPP_

#include <array>
#include <cstdint>
#include "paxi_hardware/hoverboard_protocol_struct.hpp"

namespace paxi_hardware
{

// Debug sensor data, turn on to allow publishing of hardware information
static constexpr bool DEBUG_SENSORS = true;
enum class Wheel : std::size_t
{
  LEFT = 0,
  RIGHT = 1,
  COUNT = 2
};

// Helper function to conver WheelPostion enum to appropriate index
constexpr std::size_t to_index(Wheel pos) noexcept {return static_cast<std::size_t>(pos);}

/*
     Used in templates to ensure arrays have at least two indices
     and useful for arrays storing wheel data
*/
static constexpr std::size_t WHEEL_COUNT = static_cast<std::size_t>(Wheel::COUNT);

/*
     Comes from sidebaord imu processing and madwick algorithm,
     used to scale quaternion down to correct unit
*/
static constexpr double Q30 = 1073741824.0;
/*
     Converts raw acceleration data (m/s^2) from the MPU6050 to approriate
     gravity units (16,384 LSB/g)
*/
static constexpr double ACCEL_TO_G = 16384.00;
// converts raw gyro data (DPS units) from the MPU6050 to degree persecond (16.4 LSB/(degree/s))
static constexpr double GYRO_TO_DEG_S = 16.4;

/*
     Used to convert values recieved from controller to
     values that make more sense for the hoverboard protoco
     Experiemtnally deivved, testing cmd_vel at 0.5 m/s,
*/
static constexpr double SPEED_SCALE = 5.0;
static constexpr double STEER_SCALE = 637.0;

// Based on hardware config and the way robot is build, steer needs to be flipped to match ros
// conventions of left/right turn
static constexpr double FLIP_STEER_DIRECTION = -1.0;

// Useful math stuff
static const double PI = 3.14159265358979323846;
static const double RPM_TO_RAD_S = PI / 30.0;

/*
     Internal buffer reads a sample of uint_8t feedback data into a buffer,
     256 more than enought, each feedback stuct is about ~44 bytes, so can fit like 20+,
     too small and will miss some data because we aren't checking previous indices
*/

static constexpr std::size_t CONTROLLER_FEEDBACK_BUFFER = 256;
static constexpr uint16_t K_START_FRAME = 0xABCD;

/*
     Logger names for each class, easier to debug RCLCPP_INFO/DEBUG/ERROR
*/

inline constexpr const char * LOGGER_ENCODER = "paxi_hardware_encoder";
inline constexpr const char * LOGGER_PROTOCOL = "paxi_hardware_protocol";
inline constexpr const char * LOGGER_IMU = "paxi_hardware_imu";
inline constexpr const char * LOGGER_HARDWARE = "paxi_hardware";
inline constexpr const char * LOGGER_PROTOCOL_WORKER = "paxi_hardware_protocol_worker";
inline constexpr const char * LOGGER_SERIAL = "paxi_hardware_serial";

/*
    failure handler
*/
static constexpr std::size_t MAX_NO_DATA_READS = 10;
static constexpr std::size_t MAX_DISCONNECTED_READS = 10;
static constexpr std::size_t MAX_RETRY_WRITE_COMMAND = 3;
inline constexpr double MAX_FAILURE_READ_WINDOW_SEC = 1.0;
inline constexpr std::size_t READ_RETRY_DELAY_MICROSEC = 500;

}  // namespace paxi_hardware

#endif  // PAXI_HARDWARE__UTILITY_HPP_
