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

#ifndef PAXI_COMMON__HARDWARE_TOPIC_NAMES_HPP_
#define PAXI_COMMON__HARDWARE_TOPIC_NAMES_HPP_

namespace paxi_common::hardware_topics
{

// Left wheel topic name for current PWM signal being sent to wheel motor on the firmware
inline constexpr const char * TOPIC_LEFT_FEEDBACK_CMD = "l_wheel/feedback_cmd";

// Right wheel topic name for current PWM signal being sent to wheel motor on the firmware
inline constexpr const char * TOPIC_RIGHT_FEEDBACK_CMD = "r_wheel/feedback_cmd";

// Left wheel topic name for current PWM signal being sent to frimware from hardware_interface
inline constexpr const char * TOPIC_LEFT_HARDWARE_CMD = "l_wheel/hw_cmd";

// Right wheel topic name for current PWM signal being sent to frimware from hardware_interface
inline constexpr const char * TOPIC_RIGHT_HARDWARE_CMD = "r_wheel/hw_cmd";

// Left wheel topic name for total wheel distnace travelled in meters
inline constexpr const char * TOPIC_LEFT_WHEEL_POS = "l_wheel/pos";

// Right wheel topic name for total  heel distnace travelled in meters
inline constexpr const char * TOPIC_RIGHT_WHEEL_POS = "r_wheel/pos";

// Left wheel topic name for curent  wheel velocity  in RPM
inline constexpr const char * TOPIC_LEFT_WHEEL_VEL = "l_wheel/vel";

// Right wheel topic name for current left wheel velocity in RPM
inline constexpr const char * TOPIC_RIGHT_WHEEL_VEL = "r_wheel/vel";

// Left wheel topic name for command differential drive controller is sending rad/s
inline constexpr const char * TOPIC_LEFT_CONTROLLER_CMD = "l_wheel/cmd_controller";

// Right wheel topic name command differential drive controller is sending rad/s
inline constexpr const char * TOPIC_RIGHT_CONTROLLER_CMD = "r_wheel/cmd_controller";

// Topic name for command data structure from hardware_interface (converted from diff
// drive controller)
inline constexpr const char * TOPIC_CONTROLLER_CMD = "cmd_controller";

// Topic name for feedback data structure from hoverboard MCU
inline constexpr const char * TOPIC_HOVER_FEEDBACK = "hover/feedback";

// Left wheel topic name for IMU data from hoverboard
inline constexpr const char * TOPIC_IMU_RAW = "paxi/imu_raw";

// Topic name for battery voltage for hoverboard from the Li-ion battery
inline constexpr const char * TOPIC_HOVER_BATTERY_VOLTAGE = "hover/battery_voltage";

// topic name for current termperature of the hoverboard MCU
inline constexpr const char * TOPIC_HOVER_TEMP = "hover/temperature";

// topic name for checking if USB is disconnected for the hoverboard
inline constexpr const char * TOPIC_HOVER_CONNECTED = "hover/connected";

}  // namespace paxi_common::hardware_topics

#endif  // PAXI_COMMON__HARDWARE_TOPIC_NAMES_HPP_
