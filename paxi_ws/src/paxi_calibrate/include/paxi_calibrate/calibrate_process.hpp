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

#ifndef PAXI_CALIBRATE__CALIBRATE_PROCESS_HPP_
#define PAXI_CALIBRATE__CALIBRATE_PROCESS_HPP_

#include <memory>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "paxi_calibrate/calibrate_calculations.hpp"
#include "paxi_calibrate/calibrate_csv_generator.hpp"
#include "paxi_calibrate/calibrate_subscriber.hpp"
#include "paxi_calibrate/calibrate_twist_pub.hpp"
#include "paxi_calibrate/utility.hpp"
#include "paxi_common/calibrate_logger_names.hpp"
#include "rclcpp/rclcpp.hpp"

using std::chrono_literals::operator""ms;

class CalibrateProcess : public rclcpp::Node
{
public:
  CalibrateProcess();
  ~CalibrateProcess() = default;

  void generate_tests();
  void add_linear_test(double sign);
  void add_angular_test(double sign);
  void add_linear_and_angular_test(double sign);
  void add_pause_test();

  void run_test_callback();

  const std::shared_ptr<CalibrateSubscriber> inline & get_cal_sub() const noexcept
  {
    return cal_sub_;
  }
  const std::shared_ptr<TwistPub> inline & get_cal_pub() const noexcept {return cal_pub_;}

private:
  std::shared_ptr<CalibrateSubscriber> cal_sub_;
  std::shared_ptr<TwistPub> cal_pub_;

  CalibrateCalculations cal_calc_l_;
  CalibrateCalculations cal_calc_r_;

  CSVGenerator csv_l_;
  CSVGenerator csv_r_;

  rclcpp::TimerBase::SharedPtr process_timer_;
  std::vector<std::pair<double, double>> linear_angular_tests_;

  static constexpr const char * LEFT_FILENAME = "left_wheel.csv";
  static constexpr const char * RIGHT_FILENAME = "right_wheel.csv";

  // start range for linear test eg. 1 -> (0.1, END_RANGE)
  static constexpr int START_RANGE_LINEAR = 5;

  // start range for angular test eg. 1 -> (0.1, END_RANGE)
  static constexpr int START_RANGE_ANGULAR = 5;

  // end range for linear test eg, betteween (0.00 ,0.6) example with specific test {0.11,0.59}
  static constexpr int LINEAR_TEST_END_RANGE = 10;

  // end range for linear test eg, betteween (0.00 ,0.6) example with specific test {0.11,0.59}
  static constexpr int ANGULAR_TEST_END_RANGE = 15;

  // how much to increment each test casee, e.eg 0.01 -> 0.11,0.12...,0.19 etc.
  static constexpr double STEP_COUNT_LINEAR = 0.01;

  // how much to increment each test casee, e.eg 0.01 -> 0.11,0.12...,0.19 etc.
  static constexpr double STEP_COUNT_ANGULAR = 0.1;

  // test value decimal number e.g. 10 -> 0,1  100 0.01
  static constexpr double GRANULARITY = 10.0;

  // send test {0.0,0.0} so firmware has time to get to RPM due to rate limiter
  static constexpr int PAUSE_COUNT = 5;
};
#endif  // PAXI_CALIBRATE__CALIBRATE_PROCESS_HPP_
