#include <rclcpp/rclcpp.hpp>
#include "paxi_data_collection/hardware_bag.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<paxi_data_collection::HardwareBag>());
  rclcpp::shutdown();
  return 0;
}
