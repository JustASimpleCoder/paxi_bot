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

#ifndef PAXI_CALIBRATE__CALIBRATE_TWIST_PUB_HPP_
#define PAXI_CALIBRATE__CALIBRATE_TWIST_PUB_HPP_


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


#include "paxi_calibrate/utility.hpp"
#include "paxi_common/calibrate_logger_names.hpp"

class TwistPub : public rclcpp::Node
{
public:
  TwistPub();
  ~TwistPub() = default;

  void set_linear_and_angular(double linear, double angular);
  void publish_twist();

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  geometry_msgs::msg::Twist twist_msg_;
  rclcpp::TimerBase::SharedPtr timer_pub_;
  mutable std::mutex mtx_pub_;

  // topic name for twist publisher
  static constexpr const char * TOPIC_CMD_VEL = "/cmd_vel";

};
#endif  // PAXI_CALIBRATE__CALIBRATE_TWIST_PUB_HPP_
