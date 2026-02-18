#ifndef PAXI_CALIBRATE__CALIBRATE_CPP_
#define PAXI_CALIBRATE__CALIBRATE_CPP_

#include <istream>
#include <string>

#include "paxi_calibrate/calibrate_test.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(
    rclcpp::get_logger(LOGGER_MAIN),
    "Starting Calulcations, please ensure robot is not able to move\n"
    "Enter [n] or [N] to quit anything else to start"
  );

  std::string stop;
  std::getline(std::cin, stop);
  if (stop == "n" || stop == "N") {
    rclcpp::shutdown();
    return 0;
  }

  std::shared_ptr<CalibrateTest> test = std::make_shared<CalibrateTest>();
  rclcpp::executors::MultiThreadedExecutor cal_executor;

  cal_executor.add_node(test->get_cal_sub());
  cal_executor.add_node(test->get_cal_pub());
  cal_executor.add_node(test);

  cal_executor.spin();

  rclcpp::on_shutdown(
    [&]()->int
    {
      cal_executor.cancel();
      rclcpp::shutdown();
      return 0;
    }
  );

  rclcpp::shutdown();

  return 0;
}


#endif  // PAXI_CALIBRATE__CALIBRATE_CPP_
