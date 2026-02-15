#ifndef PAXI_CALIBRATE__CALIBRATE_TEST_HPP_
#define PAXI_CALIBRATE__CALIBRATE_TEST_HPP_

#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "paxi_calibrate/utility.hpp"
#include "paxi_calibrate/calibrate_subscriber.hpp"
#include "paxi_calibrate/calibrate_calculations.hpp"
#include "paxi_calibrate/calibrate_twist_pub.hpp"
#include "paxi_calibrate/calibrate_csv_generator.hpp"

using namespace std::chrono_literals;

class CalibrateTest : public rclcpp::Node
{
public:
  CalibrateTest();
  ~CalibrateTest() = default;

  void run_test_callback();

  const std::shared_ptr<CalibrateSubscriber> inline & get_cal_sub() const noexcept 
  {
    return cal_sub;
  }
  const std::shared_ptr<TwistPub> inline & get_cal_pub() const noexcept 
  {
    return cal_pub;
  }

private:
  std::shared_ptr<CalibrateSubscriber> cal_sub;
  std::shared_ptr<TwistPub> cal_pub;

  CalibrateCalculations cal_calc;
  CSVGenerator csv;

  rclcpp::TimerBase::SharedPtr test_timer_;
};
#endif  // PAXI_CALIBRATE__CALIBRATE_NODE_HPP_
