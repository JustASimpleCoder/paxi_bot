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

#ifndef PAXI_CALIBRATE__CALIBRATE_CALCULATIONS_HPP_
#define PAXI_CALIBRATE__CALIBRATE_CALCULATIONS_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <array>
#include <cstdint>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "paxi_msgs/msg/controller_command.hpp"
#include "paxi_msgs/msg/feedback.hpp"

#include "paxi_calibrate/utility.hpp"
#include "paxi_common/calibrate_logger_names.hpp"

class CalibrateCalculations
{
public:
  CalibrateCalculations();
  ~CalibrateCalculations() = default;

  void calculate(const std::vector<double> & target, const std::vector<double> & feedback);

  void calculate_difference(
    const std::vector<double> & target, const std::vector<double> & feedback,
    std::vector<double> & difference
  );

  void calculate_ft_constant(
    const std::vector<double> & target, const std::vector<double> & feedback,
    std::vector<double> & difference
  );

  void calculate_tf_constant(
    const std::vector<double> & target, const std::vector<double> & feedback,
    std::vector<double> & difference
  );

  void print_error_msg(
    std::size_t target_size, std::size_t feedback_size,
    const std::string & calling_func
  );

  void reset_constants();

  const std::vector<double> & get_diffference() {return rpm_difference_;}
  const std::vector<double> & get_ft() {return rpm_ft_constant_;}
  const std::vector<double> & get_tf() {return rpm_tf_constant_;}


private:
  std::vector<double> rpm_difference_;   // difference vector
  std::vector<double> rpm_ft_constant_;  // ratio of RPM_feedback / RPM_Target
  std::vector<double> rpm_tf_constant_;  // ratio of RPM_Target / RPM_feedback

};
#endif  // PAXI_CALIBRATE__CALIBRATE_CALCULATIONS_HPP_
