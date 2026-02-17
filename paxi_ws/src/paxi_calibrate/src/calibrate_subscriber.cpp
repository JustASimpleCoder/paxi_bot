#include "paxi_calibrate/calibrate_subscriber.hpp"

CalibrateSubscriber::CalibrateSubscriber()
: Node("calibrate_subscriber"),
  cmd_sub_{},
  feedback_sub_{},
  got_max_samples_feedback_{false},
  got_max_samples_target_{false},
  l_target_rpm_{},
  r_target_rpm_{},
  l_feedback_rpm_{},
  r_feedback_rpm_{},
  l_target_rpm_buf_{},
  r_target_rpm_buf_{},
  l_feedback_rpm_buf_{},
  r_feedback_rpm_buf_{},
  feedback_mutex_{},
  target_mutex_{}
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

  l_target_rpm_.reserve(SAMPLE_SIZE_RPM);
  r_target_rpm_.reserve(SAMPLE_SIZE_RPM);
  l_feedback_rpm_.reserve(SAMPLE_SIZE_RPM);
  r_feedback_rpm_.reserve(SAMPLE_SIZE_RPM);

  l_target_rpm_buf_.reserve(SAMPLE_SIZE_RPM);
  r_target_rpm_buf_.reserve(SAMPLE_SIZE_RPM);
  l_feedback_rpm_buf_.reserve(SAMPLE_SIZE_RPM);
  r_feedback_rpm_buf_.reserve(SAMPLE_SIZE_RPM);
}

void CalibrateSubscriber::target_rpm_callback(
  const paxi_msgs::msg::ControllerCommand & controller_cmd)
{
  std::scoped_lock<std::mutex> lock(target_mutex_);

  if (l_target_rpm_.size() < SAMPLE_SIZE_RPM ) {
    l_target_rpm_.push_back(controller_cmd.l_speed * RAD_S_TO_RPM);
  }

  if(r_target_rpm_.size() < SAMPLE_SIZE_RPM) {
    r_target_rpm_.push_back(controller_cmd.r_speed * RAD_S_TO_RPM);
  }

  if(l_target_rpm_.size() >= SAMPLE_SIZE_RPM && r_target_rpm_.size() >= SAMPLE_SIZE_RPM){
    got_max_samples_target_ = true;
  }
}

void CalibrateSubscriber::feedback_rpm_callback(const paxi_msgs::msg::Feedback & feedback)
{
  std::scoped_lock<std::mutex> lock(feedback_mutex_);

  if (l_feedback_rpm_.size() < SAMPLE_SIZE_RPM ) {
    l_feedback_rpm_.push_back(feedback.speed_l_meas * RAD_S_TO_RPM);
  }

  if(r_feedback_rpm_.size() < SAMPLE_SIZE_RPM) {
    r_feedback_rpm_.push_back(feedback.speed_r_meas * RAD_S_TO_RPM);
  }

  if(l_feedback_rpm_.size() >= SAMPLE_SIZE_RPM && r_feedback_rpm_.size() >= SAMPLE_SIZE_RPM){
    got_max_samples_feedback_ = true;
  }

}

void CalibrateSubscriber::reset_samples() 
{
  reset_target_samples();
  reset_feedback_samples();
}


void CalibrateSubscriber::reset_target_samples() 
{
  std::scoped_lock<std::mutex> lock(target_mutex_);
  l_target_rpm_buf_ = std::move(l_target_rpm_);
  r_target_rpm_buf_ = std::move(r_target_rpm_);

  l_target_rpm_.reserve(SAMPLE_SIZE_RPM);
  r_target_rpm_.reserve(SAMPLE_SIZE_RPM);

  got_max_samples_target_ = false;
}

void CalibrateSubscriber::reset_feedback_samples() 
{
  std::scoped_lock<std::mutex> lock(feedback_mutex_);
  l_feedback_rpm_buf_ = std::move(l_feedback_rpm_);
  r_feedback_rpm_buf_ = std::move(r_feedback_rpm_);

  l_feedback_rpm_.reserve(SAMPLE_SIZE_RPM);
  r_feedback_rpm_.reserve(SAMPLE_SIZE_RPM);

  got_max_samples_feedback_ = false;
}

