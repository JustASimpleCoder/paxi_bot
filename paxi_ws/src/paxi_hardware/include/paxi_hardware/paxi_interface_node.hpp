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

#include <memory>
#include <vector>

#include "paxi_msgs/msg/controller_command.hpp"
#include "paxi_msgs/msg/feedback.hpp"

#include "paxi_hardware/hoverboard_protocol_struct.hpp"
#include "paxi_hardware/utility.hpp"
#include "paxi_common/"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

namespace paxi_hardware
{
class PaxiInterfaceNode : public rclcpp::Node
{
  // TODO(Jacob): switch this to using ImuMsg = sensor_msgs::msg::Imu;

  typedef sensor_msgs::msg::Imu ImuMsg;
  typedef std_msgs::msg::Float64 Float64Msg;
  typedef std_msgs::msg::Bool BoolMsg;

  typedef paxi_msgs::msg::ControllerCommand ControllerCmdMsg;
  typedef paxi_msgs::msg::Feedback FeedbackMsg;

public:
  PaxiInterfaceNode();
  ~PaxiInterfaceNode() = default;

  template<typename MsgT>
  void publish_data(
    const std::shared_ptr<typename rclcpp::Publisher<MsgT>> & pub,
    const MsgT & msg) const
  {
    pub->publish(msg);
  }

  template<typename MsgT, typename ValueT>
  void publish_data(
    const std::shared_ptr<typename rclcpp::Publisher<MsgT>> & pub,
    const ValueT & value) const
  {
    MsgT msg;
    msg.data = value;
    pub->publish(msg);
  }

  template<typename MsgT, typename ValLeftT, typename ValRightT>
  void publish_data(
    const std::array<std::shared_ptr<typename rclcpp::Publisher<MsgT>>, WHEEL_COUNT> & pub,
    const ValLeftT & l_value,
    const ValRightT & r_value) const
  {
    static_assert(WHEEL_COUNT == 2, "Wheel count needs to be 2, please check enum class Wheel");
    publish_data(pub[to_index(Wheel::LEFT)], l_value);
    publish_data(pub[to_index(Wheel::RIGHT)], r_value);
  }

  void publish_real_time(
    const SerialFeedback & feedback, bool connected,
    const std::vector<double> & state_positions) const;

  void publish_imu_msg(const ImuMsg & imu_msg) const;
  void publish_cmd_to_hover(const SerialCommand & cmd) const;
  void publish_controller_cmd(const double l_cmd, const double r_cmd) const;
  void publish_feedback_vel(const SerialFeedback & feedback) const;
  void publish_feedback(const SerialFeedback & feedback) const;

private:
  std::array<rclcpp::Publisher<Float64Msg>::SharedPtr, WHEEL_COUNT> position_pubs_;
  std::array<rclcpp::Publisher<Float64Msg>::SharedPtr, WHEEL_COUNT> velocity_pubs_;
  std::array<rclcpp::Publisher<Float64Msg>::SharedPtr, WHEEL_COUNT> cmd_from_hover_pubs_;
  std::array<rclcpp::Publisher<Float64Msg>::SharedPtr, WHEEL_COUNT> cmd_to_hover_pubs_;

  rclcpp::Publisher<ControllerCmdMsg>::SharedPtr controller_cmd_pub_;
  rclcpp::Publisher<FeedbackMsg>::SharedPtr feedback_pub_;

  rclcpp::Publisher<ImuMsg>::SharedPtr imu_pubs_;
  rclcpp::Publisher<Float64Msg>::SharedPtr voltage_pubs_;
  rclcpp::Publisher<Float64Msg>::SharedPtr temp_pubs_;
  rclcpp::Publisher<BoolMsg>::SharedPtr connected_pubs_;
};
}    // namespace paxi_hardware

#endif  // PAXI_HARDWARE__PAXI_INTERFACE_NODE_HPP_
