#ifndef PAXI_CALIBRATE__CALIBRATE_NODE_HPP_
#define PAXI_CALIBRATE__CALIBRATE_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <array>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "paxi_msgs/msg/controller_command.hpp"
#include "paxi_msgs/msg/feedback.hpp"

#include "paxi_calibrate/utility.hpp"

class CalibrateSubsrciber : public rclcpp::Node
{
public:
  CalibrateSubsrciber();
  ~CalibrateSubsrciber() = default;

  void target_rpm_callback(const paxi_msgs::msg::ControllerCommand & contoller_cmd);
  void feedback_rpm_callback(const paxi_msgs::msg::Feedback & feedback);

private:
  rclcpp::Subscription<paxi_msgs::msg::ControllerCommand>::SharedPtr cmd_sub_;
  rclcpp::Subscription<paxi_msgs::msg::Feedback>::SharedPtr feedback_sub_;


  std::vector<double> l_cmd_msgs_rpm;  // difference vector
  std::vector<double> r_cmd_msgs_rpm; // ratio of RPM_feedback / RPM_Target 

  std::vector<double> l_feedback_rpm;  // difference vector
  std::vector<double> r_feedback_rpm; // ratio of RPM_feedback / RPM_Target 

  std::vector<double> l_rpm_difference;  // difference vector
  std::vector<double> l_rpm_ft_constant; // ratio of RPM_feedback / RPM_Target 
  std::vector<double> l_rpm_tf_constant; // ratio of RPM_Target / RPM_feedback

  std::vector<double> r_rpm_difference;  // difference vector
  std::vector<double> r_rpm_ft_constant; // ratio of RPM_feedback / RPM_Target 
  std::vector<double> r_rpm_tf_constant; // ratio of RPM_Target / RPM_feedback
};

#endif  // PAXI_CALIBRATE__CALIBRATE_NODE_HPP_
