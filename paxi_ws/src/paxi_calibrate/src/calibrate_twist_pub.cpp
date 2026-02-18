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

#include "paxi_calibrate/calibrate_twist_pub.hpp"


using namespace std::chrono_literals;

TwistPub::TwistPub()
: Node("twist_publisher"),
  twist_pub_{},
  twist_msg_{}
{
  twist_msg_.linear.x = 0.0;
  twist_msg_.linear.y = 0.0;
  twist_msg_.linear.z = 0.0;

  twist_msg_.angular.x = 0.0;
  twist_msg_.angular.y = 0.0;
  twist_msg_.angular.z = 0.0;

  twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    TOPIC_CMD_VEL,
    10
  );
  timer_pub_ = this->create_wall_timer(500ms, std::bind(&TwistPub::publish_twist, this));
}

void TwistPub::set_linear_and_angular(double linear, double angular)
{
  twist_msg_.linear.x = linear;
  twist_msg_.angular.z = angular;
}

void TwistPub::publish_twist()
{
  twist_pub_->publish(twist_msg_);
}
