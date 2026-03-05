
#ifndef DATA_COLLECTION__HARDWARE_BAG_HPP_
#define DATA_COLLECTION__HARDWARE_BAG_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <rosbag2_cpp/writer.hpp>

#include <memory>

using std::placeholders::_1;

class HardwareBag : public rclcpp::Node
{
public:
  HardwareBag();
  ~HardwareBag() = default;

private:
  void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const;
  
  std::unique_ptr<rosbag2_cpp::Writer> hardware_writer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr hardware_subscription_;

};

#endif  // DATA_COLLECTION__HARDWARE_BAG_HPP_