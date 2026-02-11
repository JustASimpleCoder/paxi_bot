#ifndef PAXI_CALIBRATE__CALIBRATE_TWIST_PUB_HPP_
#define PAXI_CALIBRATE__CALIBRATE_TWIST_PUB_HPP_


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"



#include "paxi_calibrate/utility.hpp"

class TwistPub: public rclcpp::Node
{
public:
  TwistPub();
  ~TwistPub() = default;

  void set_linear_and_angular(double linear, double angular);
  void publish_twist();

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  geometry_msgs::msg::Twist twist_msg_;
};
#endif  // PAXI_CALIBRATE__CALIBRATE_NODE_HPP_
