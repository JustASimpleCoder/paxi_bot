#ifndef PAXI_CALIBRATE__CALIBRATE_CPP_
#define PAXI_CALIBRATE__CALIBRATE_CPP_

#include "paxi_calibrate/calibrate_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CalibrateSubsrciber>());
  rclcpp::shutdown();

  return 0;
}


#endif  // PAXI_CALIBRATE__CALIBRATE_CPP_
