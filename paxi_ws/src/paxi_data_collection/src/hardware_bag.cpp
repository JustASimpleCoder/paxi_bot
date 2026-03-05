#include "paxi_data_collection/hardware_bag.hpp"

HardwareBag::HardwareBag()
: Node("hardware_bag")
{
  hardware_writer_ = std::make_unique<rosbag2_cpp::Writer>();
  hardware_writer_->open("my_hardware_bag");

  hardware_subscription_ = create_subscription<std_msgs::msg::String>(
    "hardware_chatter", 10, std::bind(&HardwareBag::topic_callback, this, _1)
  );
}

void HardwareBag::topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
  {
    rclcpp::Time time_stamp = this->now();

    hardware_writer_->write(msg, "hardware_chatter", "std_msgs/msg/String", time_stamp);
  }