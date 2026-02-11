#ifndef PAXI_CALIBRATE__CALIBRATE_NODE_HPP_
#define PAXI_CALIBRATE__CALIBRATE_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <array>
#include <cstdint>
#include <vector>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "paxi_msgs/msg/controller_command.hpp"
#include "paxi_msgs/msg/feedback.hpp"

#include "paxi_calibrate/utility.hpp"

class CalibrateSubscriber : public rclcpp::Node
{
public:
  CalibrateSubscriber();
  ~CalibrateSubscriber() = default;

  void target_rpm_callback(const paxi_msgs::msg::ControllerCommand & contoller_cmd);
  void feedback_rpm_callback(const paxi_msgs::msg::Feedback & feedback);

  bool inline get_has_max_sample() const noexcept 
  {
    return (got_max_samples_feedback_l_ && got_max_samples_feedback_r_ &&
      got_max_samples_target_l_ && got_max_samples_target_r_) ? true : false;
  }

  std::vector<double> inline get_l_cmd_samples() const noexcept {return l_cmd_rpm_;}
  std::vector<double> inline get_r_cmd_samples() const noexcept {return r_cmd_rpm_;}
  std::vector<double> inline get_l_feedback_samples() const noexcept {return l_feedback_rpm_;}
  std::vector<double> inline get_r_feedback_samples() const noexcept {return r_feedback_rpm_;} 
private:
  rclcpp::Subscription<paxi_msgs::msg::ControllerCommand>::SharedPtr cmd_sub_;
  rclcpp::Subscription<paxi_msgs::msg::Feedback>::SharedPtr feedback_sub_;

  std::atomic<bool> got_max_samples_feedback_l_;
  std::atomic<bool> got_max_samples_feedback_r_;

  std::atomic<bool> got_max_samples_target_l_;
  std::atomic<bool> got_max_samples_target_r_;

  std::vector<double> l_cmd_rpm_;     // difference vector
  std::vector<double> r_cmd_rpm_;     // ratio of RPM_feedback / RPM_Target

  std::vector<double> l_feedback_rpm_;     // difference vector
  std::vector<double> r_feedback_rpm_;     // ratio of RPM_feedback / RPM_Target
};
#endif  // PAXI_CALIBRATE__CALIBRATE_NODE_HPP_
