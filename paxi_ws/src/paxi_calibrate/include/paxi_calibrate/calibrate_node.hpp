#ifndef _CALIBRATE_SUBSCRIBER_HPP_
#define _CALIBRATE_SUBSCRIBER_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "paxi_calibrate/utility.hpp"

class CalibrateSubsrciber : public rclcpp::Node
{
  public:

    CalibrateSubsrciber();
    ~CalibrateSubsrciber() = default;

    void target_rpm_callback_l();
    void target_rpm_callback_r();
    void feedback_rpm_callback_l();
    void feedback_rpm_callback_r();
  private:
    std::array<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr, 
      to_index(Wheel::COUNT)> cmd_sub_;
    std::array<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr, 
      to_index(Wheel::COUNT)> feedback_sub_;
};

#endif  // _CALIBRATE_SUBSCRIBER_HPP_