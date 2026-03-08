#ifndef DATA_COLLECTION__HARDWARE_BAG_HPP_
#define DATA_COLLECTION__HARDWARE_BAG_HPP_


#include <memory>
#include <array>
#include <string>
#include <utility>
#include <cstdint>

#include <chrono>
#include <ctime>
#include <sstream>

#include "paxi_common/hardware_topic_names.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <rosbag2_cpp/writer.hpp>

class HardwareBag : public rclcpp::Node
{
public:
  HardwareBag();
  ~HardwareBag() = default;

private:

  const std::string get_current_date() const;

  std::unique_ptr<rosbag2_cpp::Writer> hardware_writer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr hover_imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr slamtec_imu_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
};

#endif  // DATA_COLLECTION__HARDWARE_BAG_HPP_
