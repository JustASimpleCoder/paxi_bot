#ifndef PAXI_CALIBRATE__CALIBRATE_NODE_CPP_
#define PAXI_CALIBRATE__CALIBRATE_NODE_CPP_

#include "paxi_calibrate/calibrate_node.hpp"

CalibrateSubsrciber::CalibrateSubsrciber()
: Node("calibrate_subscriber"),
  cmd_sub_{},
  feedback_sub_{},
  l_cmd_msgs_rpm{},
  r_cmd_msgs_rpm{},
  l_feedback_rpm{},
  r_feedback_rpm{},
  l_rpm_difference{},
  l_rpm_ft_constant{},
  l_rpm_tf_constant{},
  r_rpm_difference{},
  r_rpm_ft_constant{},
  r_rpm_tf_constant{}
{
  cmd_sub_ = create_subscription<paxi_msgs::msg::ControllerCommand>(
    TOPIC_CONTROLLER_CMD,
    3,
    std::bind(&CalibrateSubsrciber::target_rpm_callback, this, std::placeholders::_1)
  );

  feedback_sub_ = create_subscription<paxi_msgs::msg::Feedback>(
    TOPIC_HOVER_FEEDBACK,
    3,
    std::bind(&CalibrateSubsrciber::feedback_rpm_callback, this, std::placeholders::_1)
  );

  l_cmd_msgs_rpm.reserve(SAMPLE_SIZE_RPM);
  r_cmd_msgs_rpm.reserve(SAMPLE_SIZE_RPM);
  l_feedback_rpm.reserve(SAMPLE_SIZE_RPM);
  r_feedback_rpm.reserve(SAMPLE_SIZE_RPM);

  l_rpm_difference.reserve(SAMPLE_SIZE_RPM);
  l_rpm_ft_constant.reserve(SAMPLE_SIZE_RPM);
  l_rpm_tf_constant.reserve(SAMPLE_SIZE_RPM);

  r_rpm_difference.reserve(SAMPLE_SIZE_RPM);
  r_rpm_ft_constant.reserve(SAMPLE_SIZE_RPM);
  r_rpm_tf_constant.reserve(SAMPLE_SIZE_RPM);
}

void CalibrateSubsrciber::target_rpm_callback(const paxi_msgs::msg::ControllerCommand & controller_cmd)
{
  if(l_cmd_msgs_rpm.size() > SAMPLE_SIZE_RPM || r_cmd_msgs_rpm.size() >= SAMPLE_SIZE_RPM){
    return;
  }

  l_cmd_msgs_rpm.push_back(controller_cmd.l_speed * RAD_S_TO_RPM);
  r_cmd_msgs_rpm.push_back(controller_cmd.r_speed * RAD_S_TO_RPM);
}

void CalibrateSubsrciber::feedback_rpm_callback(const paxi_msgs::msg::Feedback & feedback)
{
  if(l_feedback_rpm.size() > SAMPLE_SIZE_RPM || r_feedback_rpm.size() >= SAMPLE_SIZE_RPM){
    return;
  }

  l_feedback_rpm.push_back(static_cast<double>(feedback.speed_l_meas));
  r_feedback_rpm.push_back(static_cast<double>(feedback.speed_r_meas));
}
#endif  // PAXI_CALIBRATE__CALIBRATE_NODE_CPP_