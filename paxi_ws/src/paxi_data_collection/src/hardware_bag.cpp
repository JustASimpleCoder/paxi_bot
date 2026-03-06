#include "paxi_data_collection/hardware_bag.hpp"

using std::placeholders::_1;


enum class topic_msg_pair_loc : std::size_t
{
  LIDAR=0,
  HOVER_IMU=1,
  SLAMTEC_IMU=2,
  JOINT_STATE=3,
};
// TODO(Jacob): make this not global
std::array<std::pair<std::string, std::string>, 4> topic_msg_pair = {{
  {"scan", "sensor_msgs::msg::Laserscan"},
  {paxi_common::hardware_topics::TOPIC_IMU_RAW, "sensor_msgs::msg::Imu"},
  {"imu/data", "sensor_msgs::msg::Imu"},
  {"joint_states", "sensor_msgs::msg::JointState"}
}};


HardwareBag::HardwareBag()
: Node("hardware_bag")
{
  hardware_writer_ = std::make_unique<rosbag2_cpp::Writer>();
  
  hardware_writer_->open("my_hardware_bag");

  // for(const auto & [topic, msg] : topic_msg_pair){
  //   hardware_writer_->create_topic(
  //     {topic, 
  //       msg,
  //       rmw_get_serialization_format(),
  //       ""
  //     }
  //   );
  // }

  lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 
    rclcpp::SensorDataQoS(),
    std::bind(&HardwareBag::topic_lidar_callback, this, _1)
  );

  hover_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    paxi_common::hardware_topics::TOPIC_IMU_RAW, 
    rclcpp::SensorDataQoS(),
    std::bind(&HardwareBag::topic_hover_imu_callback, this, _1)
  );

  slamtec_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", 
    rclcpp::SensorDataQoS(),
    std::bind(&HardwareBag::topic_slamtec_imu_callback, this, _1)
  );

  joint_state_subscription_  = create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 
    rclcpp::SystemDefaultsQoS(),
    std::bind(&HardwareBag::topic_joint_state_callback, this, _1)
  );
}

void HardwareBag::topic_lidar_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
{
  rclcpp::Time time_stamp = this->now();
  hardware_writer_->write(
    msg, 
    topic_msg_pair[static_cast<std::size_t>(topic_msg_pair_loc::LIDAR)].first, 
    topic_msg_pair[static_cast<std::size_t>(topic_msg_pair_loc::LIDAR)].second,
    time_stamp
  );
}

void HardwareBag::topic_hover_imu_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
{
  rclcpp::Time time_stamp = this->now();
    hardware_writer_->write(
    msg, 
    topic_msg_pair[static_cast<std::size_t>(topic_msg_pair_loc::LIDAR)].first, 
    topic_msg_pair[static_cast<std::size_t>(topic_msg_pair_loc::LIDAR)].second,
    time_stamp
  );

}
void HardwareBag::topic_slamtec_imu_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
{
  rclcpp::Time time_stamp = this->now();
    hardware_writer_->write(
    msg, 
    topic_msg_pair[static_cast<std::size_t>(topic_msg_pair_loc::SLAMTEC_IMU)].first, 
    topic_msg_pair[static_cast<std::size_t>(topic_msg_pair_loc::SLAMTEC_IMU)].second,
    time_stamp
  );

}
void HardwareBag::topic_joint_state_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
{
  rclcpp::Time time_stamp = this->now();
    hardware_writer_->write(
    msg, 
    topic_msg_pair[static_cast<std::size_t>(topic_msg_pair_loc::JOINT_STATE)].first, 
    topic_msg_pair[static_cast<std::size_t>(topic_msg_pair_loc::JOINT_STATE)].second,
    time_stamp
  );
}