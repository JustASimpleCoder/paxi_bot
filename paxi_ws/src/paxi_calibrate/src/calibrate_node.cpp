#include "paxi_calibrate/calibrate_node.hpp"

CalibrateSubsrciber::CalibrateSubsrciber()
: Node("calibrate_subscriber"),
  cmd_sub_{},
  feedback_sub_{}
{

    cmd_sub_[to_index(Wheel::LEFT)] = create_subscription<std_msgs::msg::Float64>(
      "l_wheel/cmd_controller",
      3,
      std::bind(&target_rpm_callback_l, this, std::placeholders::_1)
    );

    cmd_sub_[to_index(Wheel::RIGHT)] = create_subscription<std_msgs::msg::Float64>(
      "r_wheel/vel",
      3, 
      std::bind(&target_rpm_callback_r, this,std::placeholders::_1)
    );
    
    feedback_sub_[to_index(Wheel::LEFT)] = create_subscription<std_msgs::msg::Float64>(
      "l_wheel/vel",
      3,
      std::bind(&target_rpm_callback_l, this, std::placeholders::_1)
    );

    feedback_sub_[to_index(Wheel::RIGHT)] = create_subscription<std_msgs::msg::Float64>(
      "r_wheel/vel",
      3, 
      std::bind(&target_rpm_callback_r, this,std::placeholders::_1)
    );
}

void CalibrateSubsrciber::target_rpm_callback_l() 
{

}
void CalibrateSubsrciber::target_rpm_callback_r() 
{

}

void CalibrateSubsrciber::feedback_rpm_callback_l() 
{

}

void CalibrateSubsrciber::feedback_rpm_callback_r() 
{

}

