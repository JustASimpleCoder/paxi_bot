#include "paxi_calibrate/calibrate_subscriber.hpp"

CalibrateSubscriber::CalibrateSubscriber()
: Node("calibrate_subscriber"),
  cmd_sub_{},
  feedback_sub_{},
  got_max_samples_feedback_l_{false},
  got_max_samples_feedback_r_{false},
  got_max_samples_target_l_{false},
  got_max_samples_target_r_{false},
  l_cmd_rpm_{},
  r_cmd_rpm_{},
  l_feedback_rpm_{},
  r_feedback_rpm_{}
{
  cmd_sub_ = create_subscription<paxi_msgs::msg::ControllerCommand>(
    TOPIC_CONTROLLER_CMD,
    3,
    std::bind(&CalibrateSubscriber::target_rpm_callback, this, std::placeholders::_1)
  );

  feedback_sub_ = create_subscription<paxi_msgs::msg::Feedback>(
    TOPIC_HOVER_FEEDBACK,
    3,
    std::bind(&CalibrateSubscriber::feedback_rpm_callback, this, std::placeholders::_1)
  );

  l_cmd_rpm_.reserve(SAMPLE_SIZE_RPM);
  r_cmd_rpm_.reserve(SAMPLE_SIZE_RPM);
  l_feedback_rpm_.reserve(SAMPLE_SIZE_RPM);
  r_feedback_rpm_.reserve(SAMPLE_SIZE_RPM);
}

void CalibrateSubscriber::target_rpm_callback(
  const paxi_msgs::msg::ControllerCommand & controller_cmd)
{
  if (l_cmd_rpm_.size() < SAMPLE_SIZE_RPM ) {
    l_cmd_rpm_.push_back(controller_cmd.l_speed * RAD_S_TO_RPM);
  }else{
    got_max_samples_target_l_ = true;
  }

  if(r_cmd_rpm_.size() < SAMPLE_SIZE_RPM) {
    r_cmd_rpm_.push_back(controller_cmd.r_speed * RAD_S_TO_RPM);
  }else{
     got_max_samples_target_r_ = true;
  }
}

void CalibrateSubscriber::feedback_rpm_callback(const paxi_msgs::msg::Feedback & feedback)
{

  if (l_feedback_rpm_.size() < SAMPLE_SIZE_RPM ) {
    l_feedback_rpm_.push_back(feedback.speed_l_meas * RAD_S_TO_RPM);
  }else{
    got_max_samples_feedback_l_ = true;
  }

  if(r_feedback_rpm_.size() < SAMPLE_SIZE_RPM) {
    r_feedback_rpm_.push_back(feedback.speed_r_meas * RAD_S_TO_RPM);
  }else{
     got_max_samples_feedback_r_ = true;
  }
}

void CalibrateSubscriber::reset_samples() 
{
  l_cmd_rpm_.clear();
  r_cmd_rpm_.clear();
  l_feedback_rpm_.clear();
  r_feedback_rpm_.clear();
  
  got_max_samples_feedback_l_ = false;
  got_max_samples_feedback_r_ = false;
  got_max_samples_target_l_ = false;
  got_max_samples_target_r_ = false;
}

