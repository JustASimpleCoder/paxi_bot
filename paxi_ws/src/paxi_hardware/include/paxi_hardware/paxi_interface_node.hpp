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

#ifndef PAXI_HARDWARE__PAXI_INTERFACE_NODE_HPP_
#define PAXI_HARDWARE__PAXI_INTERFACE_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "paxi_common/hardware_topic_names.hpp"
#include "paxi_common/utils.hpp"
#include "paxi_hardware/hoverboard_protocol_struct.hpp"
#include "paxi_hardware/utility.hpp"
#include "paxi_msgs/msg/controller_command.hpp"
#include "paxi_msgs/msg/feedback.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

namespace paxi_hardware
{
namespace wheel = paxi_common::utils;

/*
* Handles publishing hardware data
*/
class PaxiInterfaceNode : public rclcpp::Node
{
  using ImuMsg = sensor_msgs::msg::Imu;
  using Float64Msg = std_msgs::msg::Float64;
  using BoolMsg = std_msgs::msg::Bool;

  using ControllerCmdMsg = paxi_msgs::msg::ControllerCommand;
  using FeedbackMsg = paxi_msgs::msg::Feedback;

public:
  PaxiInterfaceNode();
  ~PaxiInterfaceNode() = default;

  void publish_real_time(
    const SerialFeedback & feedback, bool connected,
    const std::vector<double> & state_positions) const;

  void publish_imu_msg(const ImuMsg & imu_msg) const;

  // void init_imu_msg(const ImuMsg & imu_msg);
  // void update_imu_msg(const ImuMsg & imu_msg);

  void publish_cmd_to_hover(const SerialCommand & cmd) const;
  void publish_controller_cmd(const double l_cmd, const double r_cmd) const;
  void publish_feedback(const SerialFeedback & feedback) const;

private:
  std::array<rclcpp::Publisher<Float64Msg>::SharedPtr, wheel::WHEEL_COUNT> position_pubs_;
  std::array<rclcpp::Publisher<Float64Msg>::SharedPtr, wheel::WHEEL_COUNT> velocity_pubs_;
  std::array<rclcpp::Publisher<Float64Msg>::SharedPtr, wheel::WHEEL_COUNT> cmd_from_hover_pubs_;
  std::array<rclcpp::Publisher<Float64Msg>::SharedPtr, wheel::WHEEL_COUNT> cmd_to_hover_pubs_;

  rclcpp::Publisher<ControllerCmdMsg>::SharedPtr controller_cmd_pub_;
  rclcpp::Publisher<FeedbackMsg>::SharedPtr feedback_pub_;

  rclcpp::Publisher<ImuMsg>::SharedPtr imu_pub_;
  rclcpp::TimerBase::ConstSharedPtr imu_timer_;
  sensor_msgs::msg::Imu imu_msg_;

  std::mutex imu_mtx_;

  rclcpp::Publisher<Float64Msg>::SharedPtr voltage_pub_;
  rclcpp::Publisher<Float64Msg>::SharedPtr temp_pub_;
  rclcpp::Publisher<BoolMsg>::SharedPtr connected_pub_;

  // template to create std_msg from generic value and publish
  template<typename MsgT, typename ValueT>
  void publish_data(
    const std::shared_ptr<typename rclcpp::Publisher<MsgT>> & pub, const ValueT & value) const
  {
    MsgT msg;
    msg.data = value;
    pub->publish(msg);
  }

  // template to publish an arr of pubs with left and wirght wheel data
  template<typename MsgT, typename ValLeftT, typename ValRightT>
  void publish_data(
    const std::array<std::shared_ptr<typename rclcpp::Publisher<MsgT>>, wheel::WHEEL_COUNT> & pubs,
    const ValLeftT & l_value, const ValRightT & r_value) const
  {
    static_assert(
      wheel::WHEEL_COUNT == 2, "Wheel count needs to be 2, please check enum class Wheel");
    publish_data(pubs[wheel::to_index(wheel::Wheel::LEFT)], l_value);
    publish_data(pubs[wheel::to_index(wheel::Wheel::RIGHT)], r_value);
  }

  template<typename MsgT, typename ValueT>
  void debug_publish_data(
    const std::shared_ptr<typename rclcpp::Publisher<MsgT>> & pub, const ValueT & value) const
  {
    if constexpr (DEBUG_SENSORS) {
      publish_data(pub, value);
    }
  }

  template<typename MsgT, typename ValLeftT, typename ValRightT>
  void debug_publish_data(
    const std::array<std::shared_ptr<typename rclcpp::Publisher<MsgT>>, wheel::WHEEL_COUNT> & pubs,
    const ValLeftT & l_value, const ValRightT & r_value) const
  {
    if constexpr (DEBUG_SENSORS) {
      publish_data(pubs, l_value, r_value);
    }
  }
};
}  // namespace paxi_hardware

#endif  // PAXI_HARDWARE__PAXI_INTERFACE_NODE_HPP_
