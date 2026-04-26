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
#ifndef PAXI_INTERFACE_NODE_TEST_HPP_
#define PAXI_INTERFACE_NODE_TEST_HPP_

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <memory>
#include <vector>
#include <utility>
#include <string>

#include "paxi_hardware/paxi_interface_node.hpp"
#include "paxi_common/utils.hpp"
#include "test/hoverboard_protocol_struct_test.hpp"

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace paxi_hardware
{

using paxi_common::utils::to_index;
using paxi_common::utils::Wheel;
using paxi_hardware::SerialFeedback;

namespace topics = paxi_common::hardware_topics;


class TestingSubscriptionNode : public rclcpp::Node
{
  using ImuMsg = sensor_msgs::msg::Imu;
  using Float64Msg = std_msgs::msg::Float64;
  using BoolMsg = std_msgs::msg::Bool;
  using ControllerCmdMsg = paxi_msgs::msg::ControllerCommand;
  using FeedbackMsg = paxi_msgs::msg::Feedback;

public:
  TestingSubscriptionNode();
  ~TestingSubscriptionNode() = default;

  const Float64Msg & get_last_left_position_msg_() const {return last_left_position_msg_;}
  const Float64Msg & get_last_right_position_msg_() const {return last_right_position_msg_;}
  const Float64Msg & get_last_left_velocity_msg_() const {return last_left_velocity_msg_;}
  const Float64Msg & get_last_right_velocity_msg_() const {return last_right_velocity_msg_;}
  const Float64Msg & get_last_left_cmd_from_hover_msg_() const
  {
    return last_left_cmd_from_hover_msg_;
  }
  const Float64Msg & get_last_right_cmd_from_hover_msg_() const
  {
    return last_right_cmd_from_hover_msg_;
  }
  const Float64Msg & get_last_left_cmd_to_hover_msg_() const {return last_left_cmd_to_hover_msg_;}
  const Float64Msg & get_last_right_cmd_to_hover_msg_() const {return last_right_cmd_to_hover_msg_;}

  const ControllerCmdMsg get_last_controller_msg_() const {return last_controller_msg_;}
  const FeedbackMsg get_last_feedback_msg_() const {return last_feedback_msg_;}
  const ImuMsg get_last_imu_msg_() const {return last_imu_msg_;}

  const Float64Msg & get_last_voltage_msg_() const {return last_voltage_msg_;}
  const Float64Msg & get_last_temp_msg_() const {return last_temp_msg_;}
  const BoolMsg & get_last_connected_msg_() const {return last_connected_msg_;}

private:
  std::array<rclcpp::Subscription<Float64Msg>::ConstSharedPtr, wheel::WHEEL_COUNT> position_subs_;
  std::array<rclcpp::Subscription<Float64Msg>::ConstSharedPtr, wheel::WHEEL_COUNT> velocity_subs_;
  std::array<rclcpp::Subscription<Float64Msg>::ConstSharedPtr,
    wheel::WHEEL_COUNT> cmd_from_hover_subs_;
  std::array<rclcpp::Subscription<Float64Msg>::ConstSharedPtr,
    wheel::WHEEL_COUNT> cmd_to_hover_subs_;

  rclcpp::Subscription<ControllerCmdMsg>::ConstSharedPtr controller_cmd_sub_;
  rclcpp::Subscription<FeedbackMsg>::ConstSharedPtr feedback_sub_;

  rclcpp::Subscription<ImuMsg>::ConstSharedPtr imu_sub_;

  rclcpp::Subscription<Float64Msg>::ConstSharedPtr voltage_sub_;
  rclcpp::Subscription<Float64Msg>::ConstSharedPtr temp_sub_;
  rclcpp::Subscription<BoolMsg>::ConstSharedPtr connected_sub_;

  Float64Msg last_left_position_msg_;
  Float64Msg last_right_position_msg_;
  Float64Msg last_left_velocity_msg_;
  Float64Msg last_right_velocity_msg_;
  Float64Msg last_left_cmd_from_hover_msg_;
  Float64Msg last_right_cmd_from_hover_msg_;
  Float64Msg last_left_cmd_to_hover_msg_;
  Float64Msg last_right_cmd_to_hover_msg_;

  ControllerCmdMsg last_controller_msg_;
  FeedbackMsg last_feedback_msg_;

  ImuMsg last_imu_msg_;

  Float64Msg last_voltage_msg_;
  Float64Msg last_temp_msg_;
  BoolMsg last_connected_msg_;
};

class PaxiInterfaceNodeTest : public ::testing::Test
{
  using ImuMsg = sensor_msgs::msg::Imu;
  using Float64Msg = std_msgs::msg::Float64;
  using BoolMsg = std_msgs::msg::Bool;
  using ControllerCmdMsg = paxi_msgs::msg::ControllerCommand;
  using FeedbackMsg = paxi_msgs::msg::Feedback;

protected:
  void SetUp() override
  {
    //  gotten from  https://robotics.stackexchange.com/questions/25178/ros2-initialize-rclcpp-within-a-class-for-tests/103509#103509
    std::string package_share_directory = ament_index_cpp::get_package_share_directory(
      "paxi_hardware");

    char * argv[] = {::strdup("paxi_interface_node_test.hpp")};

    int argc = static_cast<int>(sizeof(argv) / sizeof(argv[0])) - 1;
    if (argc < 1) {
      argc = 1;
    }

    //   Initialization
    rclcpp::init(argc, argv);
    //  free dynamically allocated memory with strdup

    for (int i = 0; i < argc; ++i) {
      delete argv[i];
    }
    //   end of https://robotics.stackexchange.com/questions/25178/ros2-initialize-rclcpp-within-a-class-for-tests/103509#103509

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    paxi_node_ = std::make_shared<paxi_hardware::PaxiInterfaceNode>();
    test_sub_node_ = std::make_shared<TestingSubscriptionNode>();

    executor_->add_node(paxi_node_);
    executor_->add_node(test_sub_node_);
  }

  void TearDown() override
  {
    executor_->cancel();
    rclcpp::shutdown();
  }

  std::shared_ptr<paxi_hardware::PaxiInterfaceNode> paxi_node_;
  std::shared_ptr<TestingSubscriptionNode> test_sub_node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
};

class PaxiInterfaceNodeSubTest : public ::testing::Test
{};

class TestRealtimePubs : public PaxiInterfaceNodeTest,
  public ::testing::WithParamInterface<TestRealTimeParams> {};

class TestCmdToPubs : public PaxiInterfaceNodeTest,
  public ::testing::WithParamInterface<std::pair<std::int16_t, std::int16_t>> {};

class TestCmdFromPubs : public PaxiInterfaceNodeTest,
  public ::testing::WithParamInterface<std::pair<double, double>> {};

class TestFeedbackPubs : public PaxiInterfaceNodeTest,
  public ::testing::WithParamInterface<SerialFeedback> {};

}  // namespace paxi_hardware

#endif  // PAXI_INTERFACE_NODE_TEST_HPP_
