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
