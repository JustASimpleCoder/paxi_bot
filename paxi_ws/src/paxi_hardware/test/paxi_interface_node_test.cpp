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

#include "test/paxi_interface_node_test.hpp"

namespace paxi_hardware
{

using paxi_common::utils::to_index;
using paxi_common::utils::Wheel;
using paxi_hardware::SerialFeedback;

namespace topics = paxi_common::hardware_topics;

TestingSubscriptionNode::TestingSubscriptionNode()
: Node("Testin_Subscription_Node")
{
  if constexpr (DEBUG_SENSORS) {
    position_subs_[to_index(Wheel::LEFT)] =
      this->create_subscription<Float64Msg>(
      topics::TOPIC_LEFT_WHEEL_POS, 3,
      [this](const Float64Msg & msg)
      {
        last_left_position_msg_ = msg;
      }
      );

    position_subs_[to_index(Wheel::RIGHT)] =
      this->create_subscription<Float64Msg>(
      topics::TOPIC_RIGHT_WHEEL_POS, 3,
      [this](const Float64Msg & msg)
      {
        last_right_position_msg_.data = msg.data;
      }
      );

    cmd_from_hover_subs_[to_index(Wheel::LEFT)] =
      this->create_subscription<Float64Msg>(
      topics::TOPIC_LEFT_FEEDBACK_CMD, 3,
      [this](const Float64Msg & msg)
      {
        last_left_cmd_from_hover_msg_.data = msg.data;
      });

    cmd_from_hover_subs_[to_index(Wheel::RIGHT)] =
      this->create_subscription<Float64Msg>(
      topics::TOPIC_RIGHT_FEEDBACK_CMD, 3,
      [this](const Float64Msg & msg)
      {
        last_right_cmd_from_hover_msg_.data = msg.data;
      });

    cmd_to_hover_subs_[to_index(Wheel::LEFT)] =
      this->create_subscription<Float64Msg>(
      topics::TOPIC_LEFT_HARDWARE_CMD, 3,
      [this](const Float64Msg & msg)
      {
        last_left_cmd_to_hover_msg_.data = msg.data;
      });

    cmd_to_hover_subs_[to_index(Wheel::RIGHT)] =
      this->create_subscription<Float64Msg>(
      topics::TOPIC_RIGHT_HARDWARE_CMD, 3,
      [this](const Float64Msg & msg)
      {
        last_right_cmd_to_hover_msg_.data = msg.data;
      });

    velocity_subs_[to_index(Wheel::LEFT)] =
      this->create_subscription<Float64Msg>(
      topics::TOPIC_LEFT_WHEEL_VEL, 3,
      [this](const Float64Msg & msg)
      {
        last_left_velocity_msg_.data = msg.data;
      });

    velocity_subs_[to_index(Wheel::RIGHT)] =
      this->create_subscription<Float64Msg>(
      topics::TOPIC_RIGHT_WHEEL_VEL, 3,
      [this](const Float64Msg & msg)
      {
        last_right_velocity_msg_.data = msg.data;
      }
      );
  }

  if constexpr (CALIBRATE_FIRMWARE) {
    controller_cmd_sub_ = this->create_subscription<ControllerCmdMsg>(
      topics::TOPIC_CONTROLLER_CMD, 3, [this](const ControllerCmdMsg & msg)
      {
        last_controller_msg_.l_speed = msg.l_speed;
        last_controller_msg_.r_speed = msg.r_speed;
      }
    );

    feedback_sub_ = this->create_subscription<FeedbackMsg>(
      topics::TOPIC_HOVER_FEEDBACK, 3,
      [this](const FeedbackMsg & msg)
      {
        last_feedback_msg_.start = msg.start;
      }
    );
  }
  // No executor in hardware interface, its only standalone node -> can only directly publish
  imu_sub_ =
    this->create_subscription<sensor_msgs::msg::Imu>(
    topics::TOPIC_IMU_RAW, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Imu & msg)
    {
      last_imu_msg_.header.frame_id = msg.header.frame_id;
      last_imu_msg_.header.stamp = msg.header.stamp;
      last_imu_msg_.orientation_covariance = msg.orientation_covariance;
      last_imu_msg_.angular_velocity_covariance = msg.angular_velocity_covariance;
      last_imu_msg_.linear_acceleration_covariance = msg.linear_acceleration_covariance;
      last_imu_msg_.angular_velocity.x = msg.angular_velocity.x;
      last_imu_msg_.angular_velocity.y = msg.angular_velocity.y;
      last_imu_msg_.angular_velocity.z = msg.angular_velocity.z;
      last_imu_msg_.linear_acceleration.x = msg.linear_acceleration.x;
      last_imu_msg_.linear_acceleration.y = msg.linear_acceleration.y;
      last_imu_msg_.linear_acceleration.z = msg.linear_acceleration.z;
      last_imu_msg_.orientation.x = msg.orientation.x;
      last_imu_msg_.orientation.y = msg.orientation.y;
      last_imu_msg_.orientation.z = msg.orientation.z;
    });

  voltage_sub_ = this->create_subscription<Float64Msg>(
    topics::TOPIC_HOVER_BATTERY_VOLTAGE, 3, [this](const Float64Msg & msg)
    {
      last_voltage_msg_.data = msg.data;
    }
  );

  temp_sub_ = this->create_subscription<Float64Msg>(
    topics::TOPIC_HOVER_TEMP, 3, [this](const Float64Msg & msg)
    {
      last_temp_msg_.data = msg.data;
    }
  );
  connected_sub_ = this->create_subscription<BoolMsg>(
    topics::TOPIC_HOVER_CONNECTED, 3, [this](const BoolMsg & msg)
    {
      last_connected_msg_.data = msg.data;
    }
  );

  last_left_position_msg_.data = 0.0;
  last_right_position_msg_.data = 0.0;
  last_left_velocity_msg_.data = 0.0;
  last_right_velocity_msg_.data = 0.0;

  last_left_cmd_from_hover_msg_.data = 0.0;
  last_right_cmd_from_hover_msg_.data = 0.0;

  last_left_cmd_to_hover_msg_.data = 0.0;
  last_right_cmd_to_hover_msg_.data = 0.0;

  last_controller_msg_.l_speed = 0.0;
  last_controller_msg_.r_speed = 0.0;


  last_feedback_msg_.start = 0;
  last_feedback_msg_.cmd_l = 0;
  last_feedback_msg_.cmd_r = 0;
  last_feedback_msg_.speed_r_meas = 0;
  last_feedback_msg_.speed_l_meas = 0;
  last_feedback_msg_.bat_voltage = 0;
  last_feedback_msg_.board_temp = 0;
  last_feedback_msg_.gyro_x = 0;
  last_feedback_msg_.gyro_y = 0;
  last_feedback_msg_.gyro_z = 0;
  last_feedback_msg_.accel_x = 0;
  last_feedback_msg_.accel_y = 0;
  last_feedback_msg_.accel_z = 0;
  last_feedback_msg_.quat_w_low = 0;
  last_feedback_msg_.quat_w_high = 0;
  last_feedback_msg_.quat_x_low = 0;
  last_feedback_msg_.quat_x_high = 0;
  last_feedback_msg_.quat_y_low = 0;
  last_feedback_msg_.quat_y_high = 0;
  last_feedback_msg_.quat_z_low = 0;
  last_feedback_msg_.quat_z_high = 0;
  last_feedback_msg_.euler_pitch = 0;
  last_feedback_msg_.euler_roll = 0;
  last_feedback_msg_.euler_yaw = 0;
  last_feedback_msg_.temperature = 0;
  last_feedback_msg_.sensors = 0;
  last_feedback_msg_.cmd_led = 0;
  last_feedback_msg_.checksum = 0;

  last_imu_msg_.header.frame_id = "imu_hover";
  last_imu_msg_.header.stamp = this->get_clock()->now();

  last_imu_msg_.orientation_covariance = {1e-7, 0, 0, 0, 1e-7, 0, 0, 0, 1e-7};
  last_imu_msg_.angular_velocity_covariance = {1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6};
  last_imu_msg_.linear_acceleration_covariance = {1e-5, 0, 0, 0, 1e-5, 0, 0, 0, 1e-5};

  last_imu_msg_.angular_velocity.x = 0.0;
  last_imu_msg_.angular_velocity.y = 0.0;
  last_imu_msg_.angular_velocity.z = 0.0;

  last_imu_msg_.linear_acceleration.x = 0.0;
  last_imu_msg_.linear_acceleration.y = 0.0;
  last_imu_msg_.linear_acceleration.z = 0.0;

  last_imu_msg_.orientation.x = 0.0;
  last_imu_msg_.orientation.y = 0.0;
  last_imu_msg_.orientation.z = 0.0;


  last_voltage_msg_.data = 0.0;
  last_temp_msg_.data = 0.0;
  last_connected_msg_.data = false;
}

TEST_F(PaxiInterfaceNodeTest, PublishImuMsg)
{
  sensor_msgs::msg::Imu imu;
  imu.header.frame_id = "imu_hover";
  imu.angular_velocity.x = 1.0;
  imu.linear_acceleration.z = 9.81;

  paxi_node_->publish_imu_msg(imu);
  executor_->spin_some(EXECUTOR_SPIN_TIMEOUT);

  const auto & received = test_sub_node_->get_last_imu_msg_();
  EXPECT_EQ(received.header.frame_id, "imu_hover");
  EXPECT_DOUBLE_EQ(received.angular_velocity.x, 1.0);
  EXPECT_DOUBLE_EQ(received.linear_acceleration.z, 9.81);
}

TEST_F(PaxiInterfaceNodeTest, PublishRealTimeVoltageAndTemp)
{
  SerialFeedback feedback{};
  feedback.bat_voltage = 36;
  feedback.board_temp = 25;

  paxi_node_->publish_real_time(feedback, true, {0.0, 0.0});
  executor_->spin_some(EXECUTOR_SPIN_TIMEOUT);

  EXPECT_DOUBLE_EQ(test_sub_node_->get_last_voltage_msg_().data, 36.0);
  EXPECT_DOUBLE_EQ(test_sub_node_->get_last_temp_msg_().data, 25.0);
  EXPECT_TRUE(test_sub_node_->get_last_connected_msg_().data);
}

TEST_F(PaxiInterfaceNodeTest, SecondPublishOverwrites)
{
  SerialFeedback feedback{};
  feedback.bat_voltage = 36;
  feedback.board_temp = 25;

  paxi_node_->publish_real_time(feedback, true, {0.0, 0.0});
  executor_->spin_some(EXECUTOR_SPIN_TIMEOUT);

  EXPECT_DOUBLE_EQ(test_sub_node_->get_last_voltage_msg_().data, 36.0);
  EXPECT_DOUBLE_EQ(test_sub_node_->get_last_temp_msg_().data, 25.0);
  EXPECT_TRUE(test_sub_node_->get_last_connected_msg_().data);


  feedback.bat_voltage = 122.0;
  feedback.board_temp = 126.0;
  paxi_node_->publish_real_time(feedback, false, {5.0, 8.0});
  executor_->spin_some(EXECUTOR_SPIN_TIMEOUT);
  EXPECT_DOUBLE_EQ(test_sub_node_->get_last_voltage_msg_().data, 122.0);
  EXPECT_DOUBLE_EQ(test_sub_node_->get_last_temp_msg_().data, 126.0);

  if constexpr (DEBUG_SENSORS) {
    EXPECT_DOUBLE_EQ(test_sub_node_->get_last_left_position_msg_().data, 5.0);
    EXPECT_DOUBLE_EQ(test_sub_node_->get_last_right_position_msg_().data, 8.0);
  }

  EXPECT_FALSE(test_sub_node_->get_last_connected_msg_().data);
}


TEST_F(PaxiInterfaceNodeTest, DebugCmdToHover)
{
  if constexpr (DEBUG_SENSORS) {
    SerialCommand cmd1;
    cmd1.l_speed = 100; cmd1.r_speed = 200;
    paxi_node_->publish_cmd_to_hover(cmd1);
    executor_->spin_some(EXECUTOR_SPIN_TIMEOUT);

    SerialCommand cmd2;
    cmd2.l_speed = -50; cmd2.r_speed = -75;
    paxi_node_->publish_cmd_to_hover(cmd2);
    executor_->spin_some(EXECUTOR_SPIN_TIMEOUT);

    EXPECT_DOUBLE_EQ(test_sub_node_->get_last_left_cmd_to_hover_msg_().data, -50.0);
    EXPECT_DOUBLE_EQ(test_sub_node_->get_last_right_cmd_to_hover_msg_().data, -75.0);
  }
}

TEST_F(PaxiInterfaceNodeTest, NoPublishMeansDefaultValues)
{
  executor_->spin_some(std::chrono::milliseconds(50));
  EXPECT_DOUBLE_EQ(test_sub_node_->get_last_voltage_msg_().data, 0.0);
  EXPECT_DOUBLE_EQ(test_sub_node_->get_last_temp_msg_().data, 0.0);
}

TEST_F(PaxiInterfaceNodeTest, ImuTimestampIsUpdatedOnPublish)
{
  sensor_msgs::msg::Imu imu{};
  imu.header.stamp = rclcpp::Time{0};  // intentionally stale

  paxi_node_->publish_imu_msg(imu);
  executor_->spin_some(EXECUTOR_SPIN_TIMEOUT);

  const auto & received = test_sub_node_->get_last_imu_msg_();
  EXPECT_NE(received.header.stamp, rclcpp::Time{0});
}


TEST_P(TestRealtimePubs, PublishRealTime)
{
  TestRealTimeParams params = GetParam();

  paxi_node_->publish_real_time(params.feedback, params.connected, params.state_positions);
  executor_->spin_some(EXECUTOR_SPIN_TIMEOUT);

  const SerialFeedback & test_feedback = params.feedback;

  ASSERT_EQ(test_feedback.bat_voltage, test_sub_node_->get_last_voltage_msg_().data);
  ASSERT_EQ(test_feedback.board_temp, test_sub_node_->get_last_temp_msg_().data);
  ASSERT_EQ(params.connected, test_sub_node_->get_last_connected_msg_().data);


  if constexpr (DEBUG_SENSORS) {
    ASSERT_EQ(test_feedback.cmd_l,  test_sub_node_->get_last_left_cmd_from_hover_msg_().data);
    ASSERT_EQ(test_feedback.cmd_r,  test_sub_node_->get_last_left_cmd_from_hover_msg_().data);

    ASSERT_EQ(test_feedback.speed_r_meas, test_sub_node_->get_last_right_velocity_msg_().data);
    ASSERT_EQ(test_feedback.speed_l_meas, test_sub_node_->get_last_left_velocity_msg_().data);
    ASSERT_EQ(
      params.state_positions[to_index(Wheel::LEFT)],
      test_sub_node_->get_last_left_position_msg_().data);
    ASSERT_EQ(
      params.state_positions[to_index(Wheel::RIGHT)],
      test_sub_node_->get_last_right_position_msg_().data);
  }
}


TEST_P(TestCmdToPubs, PublishCmdToHover)
{
  auto cmd_test = GetParam();

  SerialCommand cmd;
  cmd.l_speed = cmd_test.first;
  cmd.r_speed = cmd_test.second;

  paxi_node_->publish_cmd_to_hover(cmd);
  executor_->spin_some(EXECUTOR_SPIN_TIMEOUT);

  ASSERT_EQ(cmd_test.first, test_sub_node_->get_last_left_cmd_to_hover_msg_().data);
  ASSERT_EQ(cmd_test.second, test_sub_node_->get_last_right_cmd_to_hover_msg_().data);
}


TEST_P(TestCmdFromPubs, PublishCmdFromhover)
{
  auto params = GetParam();
  paxi_node_->publish_controller_cmd(params.first, params.second);

  executor_->spin_some();
  executor_->spin_some(EXECUTOR_SPIN_TIMEOUT);

  ASSERT_EQ(params.first, test_sub_node_->get_last_left_cmd_from_hover_msg_().data);
  ASSERT_EQ(params.second, test_sub_node_->get_last_right_cmd_from_hover_msg_().data);
}


TEST_P(TestFeedbackPubs, PublishCmdFromhover)
{
  if constexpr (CALIBRATE_FIRMWARE) {
    auto test_feedback = GetParam();
    paxi_node_->publish_feedback(test_feedback);

    executor_->spin_some();
    executor_->spin_some(EXECUTOR_SPIN_TIMEOUT);

    paxi_msgs::msg::Feedback feedback_msg = test_sub_node_->get_last_feedback_msg_();

    ASSERT_EQ(feedback_msg.start, test_feedback.start);
    ASSERT_EQ(feedback_msg.cmd_l, test_feedback.cmd_l);
    ASSERT_EQ(feedback_msg.cmd_r, test_feedback.cmd_r);
    ASSERT_EQ(feedback_msg.speed_r_meas, test_feedback.speed_r_meas);
    ASSERT_EQ(feedback_msg.speed_l_meas, test_feedback.speed_l_meas);
    ASSERT_EQ(feedback_msg.bat_voltage, test_feedback.bat_voltage);
    ASSERT_EQ(feedback_msg.board_temp, test_feedback.board_temp);
    ASSERT_EQ(feedback_msg.gyro_x, test_feedback.gyro_x);
    ASSERT_EQ(feedback_msg.gyro_y, test_feedback.gyro_y);
    ASSERT_EQ(feedback_msg.gyro_z, test_feedback.gyro_z);
    ASSERT_EQ(feedback_msg.accel_x, test_feedback.accel_x);
    ASSERT_EQ(feedback_msg.accel_y, test_feedback.accel_y);
    ASSERT_EQ(feedback_msg.accel_z, test_feedback.accel_z);
    ASSERT_EQ(feedback_msg.quat_w_low, test_feedback.quat_w_low);
    ASSERT_EQ(feedback_msg.quat_w_high, test_feedback.quat_w_high);
    ASSERT_EQ(feedback_msg.quat_x_low, test_feedback.quat_x_low);
    ASSERT_EQ(feedback_msg.quat_x_high, test_feedback.quat_x_high);
    ASSERT_EQ(feedback_msg.quat_y_low, test_feedback.quat_y_low);
    ASSERT_EQ(feedback_msg.quat_y_high, test_feedback.quat_y_high);
    ASSERT_EQ(feedback_msg.quat_z_low, test_feedback.quat_z_low);
    ASSERT_EQ(feedback_msg.quat_z_high, test_feedback.quat_z_high);
    ASSERT_EQ(feedback_msg.euler_pitch, test_feedback.euler_pitch);
    ASSERT_EQ(feedback_msg.euler_roll, test_feedback.euler_roll);
    ASSERT_EQ(feedback_msg.euler_yaw, test_feedback.euler_yaw);
    ASSERT_EQ(feedback_msg.temperature, test_feedback.temperature);
    ASSERT_EQ(feedback_msg.sensors, test_feedback.sensors);
    ASSERT_EQ(feedback_msg.cmd_led, test_feedback.cmd_led);
    ASSERT_EQ(feedback_msg.checksum, test_feedback.checksum);
  }
}

INSTANTIATE_TEST_SUITE_P(
  PublishRealTime,
  TestRealtimePubs,
  ::testing::Values(
    TestRealTimeParams(
      create_serial_feedback(
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
        22, 23, 24, 25, 26, 27, 28),
      false,
      {1, 2}
    ),
    TestRealTimeParams(
      create_serial_feedback(),
      false,
      {1, 2}
    )
  )
);

// INSTANTIATE_TEST_SUITE_P(
//   CmdTests,
//   TestCmdToPubs,
//   ::testing::Values(
//     std::make_pair(1, 1), std::make_pair(-1, 1), std::make_pair(1, -1), std::make_pair(-1, -1)));

// INSTANTIATE_TEST_SUITE_P(
//   CmdTests,
//   TestCmdFromPubs,
//   ::testing::Values(
//     std::make_pair(1.0, 1.0), std::make_pair(-1.0, 1.0), std::make_pair(1.0, -1.0),
//     std::make_pair(-1.0, -1.0)));

// INSTANTIATE_TEST_SUITE_P(
//   PublishFeedbackCalibrate,
//   TestFeedbackPubs,
//   ::testing::Values(create_serial_feedback()));

}  // namespace paxi_hardware
