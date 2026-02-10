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

PaxiInterfaceNode::PaxiInterfaceNode()
: Node("paxi_interface_node")
{
  if constexpr (DEBUG_SENSORS){
    position_pubs_[to_index(Wheel::LEFT)] =
      this->create_publisher<std_msgs::msg::Float64>("l_wheel/pos", 3);

    position_pubs_[to_index(Wheel::RIGHT)] =
      this->create_publisher<std_msgs::msg::Float64>("r_wheel/pos", 3);

    cmd_from_hover_pubs_[to_index(Wheel::LEFT)] =
      this->create_publisher<std_msgs::msg::Float64>("l_wheel/cmd_from", 3);

    cmd_from_hover_pubs_[to_index(Wheel::RIGHT)] =
      this->create_publisher<std_msgs::msg::Float64>("r_wheel/cmd_from", 3);

    cmd_to_hover_pubs_[to_index(Wheel::LEFT)] =
      this->create_publisher<std_msgs::msg::Float64>("l_wheel/cmd_to", 3);

    cmd_to_hover_pubs_[to_index(Wheel::RIGHT)] =
      this->create_publisher<std_msgs::msg::Float64>("r_wheel/cmd_to", 3);
  }

  if constexpr(DEBUG_SENSORS || CALIBRATE_FIRMWARE){
    velocity_pubs_[to_index(Wheel::LEFT)] =
      this->create_publisher<std_msgs::msg::Float64>("l_wheel/vel", 3);

    velocity_pubs_[to_index(Wheel::RIGHT)] =
      this->create_publisher<std_msgs::msg::Float64>("r_wheel/vel", 3);
  }

  if constexpr (CALIBRATE_FIRMWARE) {
    controller_cmd_pubs_[to_index(Wheel::LEFT)] =
      this->create_publisher<std_msgs::msg::Float64>("l_wheel/cmd_controller", 3);

    controller_cmd_pubs_[to_index(Wheel::RIGHT)] =
      this->create_publisher<std_msgs::msg::Float64>("r_wheel/cmd_controller", 3);
  }

  imu_pubs_ =
    this->create_publisher<sensor_msgs::msg::Imu>("paxi/imu_raw", rclcpp::SensorDataQoS());

  voltage_pubs_ = this->create_publisher<std_msgs::msg::Float64>("hover/battery_voltage", 3);
  temp_pubs_ = this->create_publisher<std_msgs::msg::Float64>("hover/temperature", 3);
  connected_pubs_ = this->create_publisher<std_msgs::msg::Bool>("hover/connected", 3);
}

void PaxiInterfaceNode::publish_real_time(
  const SerialFeedback & feedback,
  bool connected,
  const std::vector<double> & state_positions) const
{
  publish_data<std_msgs::msg::Float64>(
    cmd_from_hover_pubs_,
    feedback.cmd_l,
    feedback.cmd_r
  );

  publish_data<std_msgs::msg::Float64>(
    voltage_pubs_,
    feedback.bat_voltage
  );

  publish_data<std_msgs::msg::Float64>(
    temp_pubs_,
    feedback.board_temp
  );

  publish_data<std_msgs::msg::Float64>(
    velocity_pubs_,
    feedback.speed_l_meas,
    feedback.speed_r_meas
  );

  publish_data<std_msgs::msg::Float64>(
    position_pubs_,
    state_positions[to_index(Wheel::LEFT)],
    state_positions[to_index(Wheel::RIGHT)]
  );

  publish_data<std_msgs::msg::Bool>(
    connected_pubs_, connected
  );
}

void PaxiInterfaceNode::publish_feedback_vel(const SerialFeedback & feedback) const
{
  publish_data<std_msgs::msg::Float64>(
    velocity_pubs_,
    feedback.speed_l_meas,
    feedback.speed_r_meas
  );
}

void PaxiInterfaceNode::publish_cmd_to_hover(const SerialCommand & cmd) const
{
  publish_data<std_msgs::msg::Float64>(
    cmd_to_hover_pubs_,
    cmd.l_speed,
    cmd.r_speed
  );
}

void PaxiInterfaceNode::publish_controller_cmd(const double l_cmd, const double r_cmd) const
{
  publish_data<std_msgs::msg::Float64>(
    controller_cmd_pubs_,
    l_cmd,
    r_cmd
  );
}

void PaxiInterfaceNode::publish_imu_msg(const sensor_msgs::msg::Imu & imu_msg) const
{
  publish_data<sensor_msgs::msg::Imu>(imu_pubs_, imu_msg);
}

}  // namespace paxi_hardware
