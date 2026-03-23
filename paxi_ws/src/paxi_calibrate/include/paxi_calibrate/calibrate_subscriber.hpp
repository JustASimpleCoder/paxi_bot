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

#ifndef PAXI_CALIBRATE__CALIBRATE_SUBSCRIBER_HPP_
#define PAXI_CALIBRATE__CALIBRATE_SUBSCRIBER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "paxi_calibrate/utility.hpp"
#include "paxi_common/calibrate_logger_names.hpp"
#include "paxi_common/hardware_topic_names.hpp"
#include "paxi_common/math.hpp"
#include "paxi_msgs/msg/controller_command.hpp"
#include "paxi_msgs/msg/feedback.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class CalibrateSubscriber : public rclcpp::Node
{
public:
  CalibrateSubscriber();
  ~CalibrateSubscriber() = default;

  void target_rpm_callback(const paxi_msgs::msg::ControllerCommand & contoller_cmd);
  void feedback_rpm_callback(const paxi_msgs::msg::Feedback & feedback);
  void reset_samples();
  void reset_target_samples();
  void reset_feedback_samples();

  bool inline get_has_max_sample() const noexcept
  {
    return (got_max_samples_feedback_ && got_max_samples_target_) ? true : false;
  }

  std::vector<double> inline get_l_target_samples() const noexcept {return l_target_rpm_buf_;}
  std::vector<double> inline get_r_target_samples() const noexcept {return r_target_rpm_buf_;}
  std::vector<double> inline get_l_feedback_samples() const noexcept {return l_feedback_rpm_buf_;}
  std::vector<double> inline get_r_feedback_samples() const noexcept {return r_feedback_rpm_buf_;}

private:
  rclcpp::Subscription<paxi_msgs::msg::ControllerCommand>::SharedPtr cmd_sub_;
  rclcpp::Subscription<paxi_msgs::msg::Feedback>::SharedPtr feedback_sub_;

  std::atomic<bool> got_max_samples_feedback_;
  std::atomic<bool> got_max_samples_target_;

  std::vector<double> l_target_rpm_;  // difference vector
  std::vector<double> r_target_rpm_;  // ratio of RPM_feedback / RPM_Target

  std::vector<double> l_feedback_rpm_;  // difference vector
  std::vector<double> r_feedback_rpm_;  // ratio of RPM_feedback / RPM_Target

  std::vector<double> l_target_rpm_buf_;  // difference vector
  std::vector<double> r_target_rpm_buf_;  // ratio of RPM_feedback / RPM_Target

  std::vector<double> l_feedback_rpm_buf_;  // difference vector
  std::vector<double> r_feedback_rpm_buf_;  // ratio of RPM_feedback / RPM_Target

  std::mutex feedback_mutex_;
  std::mutex target_mutex_;
};

#endif  // PAXI_CALIBRATE__CALIBRATE_SUBSCRIBER_HPP_
