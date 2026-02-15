#ifndef PAXI_CALIBRATE__CALIBRATE_CPP_
#define PAXI_CALIBRATE__CALIBRATE_CPP_

// #include "paxi_calibrate/calibrate_subscriber.hpp"
// #include "paxi_calibrate/calibrate_calculations.hpp"
// #include "paxi_calibrate/calibrate_twist_pub.hpp"
// #include "paxi_calibrate/calibrate_csv_generator.hpp"
#include "paxi_calibrate/calibrate_test.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<CalibrateTest> test = std::make_shared<CalibrateTest>(); 
  rclcpp::executors::MultiThreadedExecutor cal_executor;

  cal_executor.add_node(test->get_cal_sub());
  cal_executor.add_node(test->get_cal_pub());
  cal_executor.add_node(test);
  cal_executor.spin();
  rclcpp::shutdown();

  return 0;
}


#endif  // PAXI_CALIBRATE__CALIBRATE_CPP_
