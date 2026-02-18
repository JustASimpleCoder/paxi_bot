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

#ifndef PAXI_CALIBRATE__UTILITY_HPP_
#define PAXI_CALIBRATE__UTILITY_HPP_

#include <cstdint>
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
     useful math stuff
*/
inline constexpr double PI = 3.14159265358979323846;
inline constexpr double RPM_TO_RAD_S = PI / 30.0;
inline constexpr double RAD_S_TO_RPM = 30.0 / PI;

/*
     sample size of data collection
*/
inline constexpr std::size_t SAMPLE_SIZE_RPM = 100;


/*
     topic names
*/
inline constexpr const char * TOPIC_CONTROLLER_CMD = "cmd_controller";
inline constexpr const char * TOPIC_HOVER_FEEDBACK = "hover/feedback";
inline constexpr const char * TOPIC_CMD_VEL = "/cmd_vel";

/*
     ros logger names
*/

inline constexpr const char * LOGGER_CALCULATION = "Calibration_Calculation";
inline constexpr const char * LOGGER_SUBSCRIBER = "Calibration_Subscriber";
inline constexpr const char * LOGGER_PUBLISHER = "Calibration_Publisher";
inline constexpr const char * LOGGER_MAIN = "Calibration_Main";

/*
     CSV file stuff
*/
// HEader for left wheel
inline constexpr const char * CSV_L_HEADER =
  "linear,angular,target_rpm,feedback_rpm,l_difference,l_tf,l_ft";

// Header for right wheel
inline constexpr const char * CSV_R_HEADER =
  "lienar,angular,target_rpm,feedback_rpm,r_difference,r_tf,r_tf";

inline constexpr const char * LEFT_FILENAME = "Left_wheel.csv";
inline constexpr const char * RIGHT_FILENAME = "right_wheel.csv";

/*
     generate test stuff
*/


// start range for linear test eg. 1 -> (0.1, END_RANGE)
inline constexpr int START_RANGE_LINEAR = 5;

// start range for angular test eg. 1 -> (0.1, END_RANGE)
inline constexpr int START_RANGE_ANGULAR = 10;

// end range for linear test eg, betteween (0.00 ,0.6) example with specific test {0.11,0.59}
inline constexpr int LINEAR_TEST_END_RANGE = 15;

// end range for linear test eg, betteween (0.00 ,0.6) example with specific test {0.11,0.59}
inline constexpr int ANGULAR_TEST_END_RANGE = 30;

// how much to increment each test casee, e.eg 0.01 -> 0.11,0.12...,0.19 etc.
inline constexpr double STEP_COUNT_LINEAR = 0.01;

// how much to increment each test casee, e.eg 0.01 -> 0.11,0.12...,0.19 etc.
inline constexpr double STEP_COUNT_ANGULAR = 1.0;

// test value decimal number e.g. 10 -> 0,1  100 0.01
inline constexpr double GRANULARITY = 10.0;

// send test {0.0,0.0} so firmware has time to get to RPM due to rate limiter
constexpr int PAUSE_COUNT = 5;


#endif  // PAXI_CALIBRATE__UTILITY_HPP_
