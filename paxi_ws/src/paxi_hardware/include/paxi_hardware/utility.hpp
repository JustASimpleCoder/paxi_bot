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

#ifndef PAXI_HARDWARE__UTILITY_HPP_
#define PAXI_HARDWARE__UTILITY_HPP_

#include <array>
#include <cstdint>
#include "paxi_hardware/hoverboard_protocol_struct.hpp"

namespace paxi_hardware
{

/*
     Debug sensor data, turn on to allow publishing of hardware information
*/
inline constexpr bool DEBUG_SENSORS = true;

/*
     Used in conjuction with calibrate package. Sets hardware to publish messages
     in order to calibrate RPM speeds
*/
inline constexpr bool CALIBRATE_FIRMWARE = false;

/*
     Enum to store wheel index and total wheel count
          LEFT = 0,
          RIGHT = 1,
          COUNT = 2
*/
enum class Wheel : std::size_t
{
  LEFT = 0,
  RIGHT = 1,
  COUNT = 2
};

/*
     Helper function to conver WheelPostion enum to appropriate index
*/
constexpr std::size_t to_index(Wheel pos) noexcept {return static_cast<std::size_t>(pos);}

/*
     Used in templates to ensure arrays have at least two indices
     and useful for arrays storing wheel data
*/
inline constexpr std::size_t WHEEL_COUNT = static_cast<std::size_t>(Wheel::COUNT);

/*
     Comes from sidebaord imu processing and madwick algorithm,
     used to scale quaternion down to correct unit
*/
inline constexpr double Q30 = 1073741824.0;
/*
     Converts raw acceleration data (m/s^2) from the MPU6050 to approriate
     gravity units (16,384 LSB/g)
*/
inline constexpr double ACCEL_TO_G = 16384.00;

/*
     converts raw gyro data (DPS units) from the MPU6050 to
     degree persecond (16.4 LSB/(degree/s))
*/
inline constexpr double GYRO_TO_DEG_S = 16.4;

/*
     Standard gravity constant 9.81 m/s^2
*/
inline constexpr double STD_GRAVITY = 9.81;


/*
     Values recieved from doing linear regression model
     using the paxi_calibrate package.

     data analysis:
     **** LEFT WHEEL DATA POS ****
     R-squared Left Pos [0.9998195592973994]
     Intercept Left Pos [40.43878357330695]
     Slope Left Pos     [[2.02618121]]

     **** LEFT WHEEL DATA NEG ****
     R-squared Left Pos [0.9998546047544311]
     Intercept Left Pos [-38.450275676263146]
     Slope Left Pos     [[2.02397546]]

     **** Right WHEEL DATA POS****
     R-squared Right Pos [0.9996574935016734]
     Intercept Right Pos [42.897116322354975]
     Slope Right Pos     [[2.02091579]]

     **** Right WHEEL DATA NEG****
     R-squared Right Pos [0.9996263861487986]
     Intercept Right Pos [-45.83463084595394]
     Slope Right Pos     [[2.01675397]]

*/

inline constexpr double L_POS_SLOPE = 2.02618121;
inline constexpr double L_NEG_SLOPE = 2.02397546;
inline constexpr double L_POS_INTERCEPT = 40.43878;
inline constexpr double L_NEG_INTERCEPT = -38.45028;

inline constexpr double R_POS_SLOPE = 2.02091579;
inline constexpr double R_NEG_SLOPE = 2.01675397;
inline constexpr double R_POS_INTERCEPT = 42.89712;
inline constexpr double R_NEG_INTERCEPT = -45.83463;


/*
     Internal buffer reads a sample of uint_8t feedback data into a buffer,
     256 more than enought, each feedback stuct is about ~44 bytes, so can fit like 20+,
     too small and will miss some data because we aren't checking previous indices
*/

inline constexpr std::size_t CONTROLLER_FEEDBACK_BUFFER = 256;
inline constexpr std::uint16_t K_START_FRAME = 0xABCD;

/*
    failure handler
*/
inline constexpr std::size_t MAX_NO_DATA_READS = 10;
inline constexpr std::size_t MAX_DISCONNECTED_READS = 10;
inline constexpr std::size_t MAX_RETRY_WRITE_COMMAND = 3;
inline constexpr double MAX_FAILURE_READ_WINDOW_SEC = 1.0;
inline constexpr std::size_t READ_RETRY_DELAY_MICROSEC = 500;

}  // namespace paxi_hardware

#endif  // PAXI_HARDWARE__UTILITY_HPP_
