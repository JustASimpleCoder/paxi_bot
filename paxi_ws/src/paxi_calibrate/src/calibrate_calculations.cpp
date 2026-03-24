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

#include "paxi_calibrate/calibrate_calculations.hpp"

using paxi_common::calibrate_loggers::LOGGER_CALCULATION;

CalibrateCalculations::CalibrateCalculations()
: rpm_difference_{}, rpm_ft_constant_{}, rpm_tf_constant_{}
{
  rpm_difference_.reserve(SAMPLE_SIZE_RPM);
  rpm_ft_constant_.reserve(SAMPLE_SIZE_RPM);
  rpm_tf_constant_.reserve(SAMPLE_SIZE_RPM);
}

void CalibrateCalculations::calculate(
  const std::vector<double> & target, const std::vector<double> & feedback)
{
  calculate_difference(target, feedback, rpm_difference_);
  calculate_tf_constant(target, feedback, rpm_tf_constant_);
  calculate_ft_constant(target, feedback, rpm_ft_constant_);
}

void CalibrateCalculations::calculate_difference(
  const std::vector<double> & target, const std::vector<double> & feedback,
  std::vector<double> & difference)
{
  if (target.size() != feedback.size()) {
    print_error_msg(target.size(), feedback.size(), "difference constant");
    return;
  }

  for (std::size_t i = 0u; i < target.size(); ++i) {
    difference.push_back(target[i] - feedback[i]);
  }
}

void CalibrateCalculations::calculate_ft_constant(
  const std::vector<double> & target, const std::vector<double> & feedback,
  std::vector<double> & difference)
{
  if (target.size() != feedback.size()) {
    print_error_msg(target.size(), feedback.size(), "f / t constant");
    return;
  }

  for (std::size_t i = 0u; i < target.size(); ++i) {
    if (target[i] == 0.0) {
      difference.push_back(std::numeric_limits<double>::quiet_NaN());
      continue;
    }
    difference.push_back(feedback[i] / target[i]);
  }
}

void CalibrateCalculations::calculate_tf_constant(
  const std::vector<double> & target, const std::vector<double> & feedback,
  std::vector<double> & difference)
{
  if (target.size() != feedback.size()) {
    print_error_msg(target.size(), feedback.size(), "t / f constant");
    return;
  }

  for (std::size_t i = 0u; i < target.size(); ++i) {
    if (feedback[i] == 0.0) {
      difference.push_back(std::numeric_limits<double>::quiet_NaN());
      continue;
    }
    difference.push_back(target[i] / feedback[i]);
  }
}

void CalibrateCalculations::print_error_msg(
  std::size_t target_size, std::size_t feedback_size, const std::string & calling_func)
{
  RCLCPP_INFO(
    rclcpp::get_logger(LOGGER_CALCULATION),
    "Vectors given are not the same size target RPM size [%ld] and feeback rpm size [%ld]"
    "for function [%s]",
    target_size, feedback_size, calling_func.c_str());
}

void CalibrateCalculations::reset_constants()
{
  rpm_difference_.clear();
  rpm_ft_constant_.clear();
  rpm_tf_constant_.clear();
}
