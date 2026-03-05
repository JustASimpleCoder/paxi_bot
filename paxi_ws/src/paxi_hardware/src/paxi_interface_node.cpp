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

#include "paxi_hardware/paxi_interface_node.hpp"
#include <functional>
namespace paxi_hardware
{

namespace topics = paxi_common::hardware_topics;

PaxiInterfaceNode::PaxiInterfaceNode()
: Node("paxi_interface_node")
{
  if constexpr (DEBUG_SENSORS) {
    position_pubs_[to_index(Wheel::LEFT)] =
      create_publisher<Float64Msg>(topics::TOPIC_LEFT_WHEEL_POS, 3);

    position_pubs_[to_index(Wheel::RIGHT)] =
      create_publisher<Float64Msg>(topics::TOPIC_RIGHT_WHEEL_POS, 3);

    cmd_from_hover_pubs_[to_index(Wheel::LEFT)] =
      create_publisher<Float64Msg>(topics::TOPIC_LEFT_FEEDBACK_CMD, 3);

    cmd_from_hover_pubs_[to_index(Wheel::RIGHT)] =
      create_publisher<Float64Msg>(topics::TOPIC_RIGHT_FEEDBACK_CMD, 3);

    cmd_to_hover_pubs_[to_index(Wheel::LEFT)] =
      create_publisher<Float64Msg>(topics::TOPIC_LEFT_HARDWARE_CMD, 3);

    cmd_to_hover_pubs_[to_index(Wheel::RIGHT)] =
      create_publisher<Float64Msg>(topics::TOPIC_RIGHT_HARDWARE_CMD, 3);

    velocity_pubs_[to_index(Wheel::LEFT)] =
      create_publisher<Float64Msg>(topics::TOPIC_LEFT_WHEEL_VEL, 3);

    velocity_pubs_[to_index(Wheel::RIGHT)] =
      create_publisher<Float64Msg>(topics::TOPIC_RIGHT_WHEEL_VEL, 3);
  }

  if constexpr (CALIBRATE_FIRMWARE) {
    controller_cmd_pub_ = create_publisher<ControllerCmdMsg>(
      topics::TOPIC_CONTROLLER_CMD,
      3
    );

    feedback_pub_ = create_publisher<FeedbackMsg>(
      topics::TOPIC_HOVER_FEEDBACK,
      3
    );
  }

  imu_pubs_ =
    create_publisher<sensor_msgs::msg::Imu>(topics::TOPIC_IMU_RAW, rclcpp::SensorDataQoS());

  voltage_pubs_ = create_publisher<Float64Msg>(topics::TOPIC_HOVER_BATTERY_VOLTAGE, 3);
  temp_pubs_ = create_publisher<Float64Msg>(topics::TOPIC_HOVER_TEMP, 3);
  connected_pubs_ = create_publisher<BoolMsg>(topics::TOPIC_HOVER_CONNECTED, 3);
}

void PaxiInterfaceNode::publish_real_time(
  const SerialFeedback & feedback,
  bool connected,
  const std::vector<double> & state_positions) const
{
  publish_data<Float64Msg>(cmd_from_hover_pubs_, feedback.cmd_l, feedback.cmd_r);

  publish_data<Float64Msg>(voltage_pubs_, feedback.bat_voltage);

  publish_data<Float64Msg>(temp_pubs_, feedback.board_temp);

  publish_data<Float64Msg>(velocity_pubs_, feedback.speed_l_meas, feedback.speed_r_meas);

  publish_data<Float64Msg>(
    position_pubs_, state_positions[to_index(Wheel::LEFT)],
    state_positions[to_index(Wheel::RIGHT)]
  );

  publish_data<BoolMsg>(connected_pubs_, connected);
}

void PaxiInterfaceNode::publish_feedback_vel(const SerialFeedback & feedback) const
{
  publish_data<Float64Msg>(velocity_pubs_, feedback.speed_l_meas, feedback.speed_r_meas);
}

void PaxiInterfaceNode::publish_cmd_to_hover(const SerialCommand & cmd) const
{
  publish_data<Float64Msg>(cmd_to_hover_pubs_, cmd.l_speed, cmd.r_speed);
}

void PaxiInterfaceNode::publish_controller_cmd(const double l_cmd, const double r_cmd) const
{
  ControllerCmdMsg controller_cmd;
  controller_cmd.l_speed = l_cmd;
  controller_cmd.r_speed = r_cmd;
  controller_cmd_pub_->publish(controller_cmd);
}
void PaxiInterfaceNode::publish_feedback(const SerialFeedback & feedback) const
{
  FeedbackMsg feedback_msg;

  feedback_msg.start = feedback.start;
  feedback_msg.cmd_l = feedback.cmd_l;
  feedback_msg.cmd_r = feedback.cmd_r;
  feedback_msg.speed_r_meas = feedback.speed_r_meas;
  feedback_msg.speed_l_meas = feedback.speed_l_meas;
  feedback_msg.bat_voltage = feedback.bat_voltage;
  feedback_msg.board_temp = feedback.board_temp;
  feedback_msg.gyro_x = feedback.gyro_x;
  feedback_msg.gyro_y = feedback.gyro_y;
  feedback_msg.gyro_z = feedback.gyro_z;
  feedback_msg.accel_x = feedback.accel_x;
  feedback_msg.accel_y = feedback.accel_y;
  feedback_msg.accel_z = feedback.accel_z;
  feedback_msg.quat_w_low = feedback.quat_w_low;
  feedback_msg.quat_w_high = feedback.quat_w_high;
  feedback_msg.quat_x_low = feedback.quat_x_low;
  feedback_msg.quat_x_high = feedback.quat_x_high;
  feedback_msg.quat_y_low = feedback.quat_y_low;
  feedback_msg.quat_y_high = feedback.quat_y_high;
  feedback_msg.quat_z_low = feedback.quat_z_low;
  feedback_msg.quat_z_high = feedback.quat_z_high;
  feedback_msg.euler_pitch = feedback.euler_pitch;
  feedback_msg.euler_roll = feedback.euler_roll;
  feedback_msg.euler_yaw = feedback.euler_yaw;
  feedback_msg.temperature = feedback.temperature;
  feedback_msg.sensors = feedback.sensors;
  feedback_msg.cmd_led = feedback.cmd_led;
  feedback_msg.checksum = feedback.checksum;

  feedback_pub_->publish(feedback_msg);
}

void PaxiInterfaceNode::publish_imu_msg(const ImuMsg & imu_msg) const
{
  publish_data<ImuMsg>(imu_pubs_, imu_msg);
}

}  // namespace paxi_hardware
