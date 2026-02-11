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

class CalibrateCalculations
{
public:
  CalibrateCalculations();
  ~CalibrateCalculations() = default;

  void calculate_l(
    const std::vector<double> & target, 
    const std::vector<double> & feedback
  );

  void calculate_r(
    const std::vector<double> & target, 
    const std::vector<double> & feedback
  );

  void calculate_difference(
    const std::vector<double> & target, 
    const std::vector<double> & feedback,
    std::vector<double> & difference
  );

  void calulcate_ft_constant(
    const std::vector<double> & target, 
    const std::vector<double> & feedback,
    std::vector<double> & difference
  );

  void calculate_tf_constant(
    const std::vector<double> & target, 
    const std::vector<double> & feedback,
    std::vector<double> & difference
  );

  void print_error_msg(std::size_t target_size, std::size_t feedback_size, 
    const std::string & calling_func);

private:
  std::vector<double> l_rpm_difference_;   // difference vector
  std::vector<double> l_rpm_ft_constant_;  // ratio of RPM_feedback / RPM_Target
  std::vector<double> l_rpm_tf_constant_;  // ratio of RPM_Target / RPM_feedback

  std::vector<double> r_rpm_difference_;   // difference vector
  std::vector<double> r_rpm_ft_constant_;  // ratio of RPM_feedback / RPM_Target
  std::vector<double> r_rpm_tf_constant_;  // ratio of RPM_Target / RPM_feedback
};
#endif  // PAXI_CALIBRATE__CALIBRATE_CALCULATIONS_HPP_
