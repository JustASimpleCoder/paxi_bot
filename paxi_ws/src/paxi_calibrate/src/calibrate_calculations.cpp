#include "paxi_calibrate/calibrate_calculations.hpp"

CalibrateCalculations::CalibrateCalculations()
: l_rpm_difference_{},
  l_rpm_ft_constant_{},
  l_rpm_tf_constant_{},
  r_rpm_difference_{},
  r_rpm_ft_constant_{},
  r_rpm_tf_constant_{}
{
  l_rpm_difference_.reserve(SAMPLE_SIZE_RPM);
  l_rpm_ft_constant_.reserve(SAMPLE_SIZE_RPM);
  l_rpm_tf_constant_.reserve(SAMPLE_SIZE_RPM);

  r_rpm_difference_.reserve(SAMPLE_SIZE_RPM);
  r_rpm_ft_constant_.reserve(SAMPLE_SIZE_RPM);
  r_rpm_tf_constant_.reserve(SAMPLE_SIZE_RPM);
}

void CalibrateCalculations::calculate_l(
  const std::vector<double> & target,
  const std::vector<double> & feedback)
{
  calculate_difference(target, feedback, l_rpm_difference_);
  calculate_tf_constant(target, feedback, l_rpm_tf_constant_);
  calculate_ft_constant(target, feedback, l_rpm_ft_constant_);
}

void CalibrateCalculations::calculate_r(
  const std::vector<double> & target,
  const std::vector<double> & feedback)
{
  calculate_difference(target, feedback, r_rpm_difference_);
  calculate_tf_constant(target, feedback, r_rpm_tf_constant_);
  calculate_ft_constant(target, feedback, r_rpm_ft_constant_);
}

void CalibrateCalculations::calculate_difference(
  const std::vector<double> & target,
  const std::vector<double> & feedback,
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
  const std::vector<double> & target,
  const std::vector<double> & feedback,
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
  const std::vector<double> & target,
  const std::vector<double> & feedback,
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
  std::size_t target_size,
  std::size_t feedback_size,
  const std::string & calling_func)
{
  RCLCPP_INFO(
    rclcpp::get_logger(LOGGER_CALCULATION),
    "Vectors given are not the same size target RPM size [%ld] and feeback rpm size [%ld]"
    "for function [%s]",
    target_size,
    feedback_size,
    calling_func.c_str()
  );
}

void CalibrateCalculations::reset_constants()
{
  l_rpm_difference_.clear();
  l_rpm_ft_constant_.clear();
  l_rpm_tf_constant_.clear();

  r_rpm_difference_.clear();
  r_rpm_ft_constant_.clear();
  r_rpm_tf_constant_.clear();
}
