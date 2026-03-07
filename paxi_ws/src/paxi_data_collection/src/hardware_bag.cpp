#include "paxi_data_collection/hardware_bag.hpp"

HardwareBag::HardwareBag()
: Node("hardware_bag")
{
  hardware_writer_ = std::make_unique<rosbag2_cpp::Writer>();
  hardware_writer_->open("my_hardware_bag");

  const rosbag2_storage::TopicMetadata lidar_topic_data = {
    "scan",
    "sensor_msgs/msg/LaserScan",
    rmw_get_serialization_format(),
    ""
  };

  const rosbag2_storage::TopicMetadata imu_hover_topic_data = {
    paxi_common::hardware_topics::TOPIC_IMU_RAW,
    "sensor_msgs/msg/Imu",
    rmw_get_serialization_format(),
    ""
  };

  const rosbag2_storage::TopicMetadata imu_slamtec_topic_data = {
    "imu/data",
    "sensor_msgs/msg/Imu",
    rmw_get_serialization_format(),
    ""
  };

  const rosbag2_storage::TopicMetadata joints_topic_data = {
    "joint_states",
    "sensor_msgs/msg/JointState",
    rmw_get_serialization_format(),
    ""
  };

  hardware_writer_->create_topic(lidar_topic_data);
  hardware_writer_->create_topic(imu_hover_topic_data);
  hardware_writer_->create_topic(imu_slamtec_topic_data);
  hardware_writer_->create_topic(joints_topic_data);

  auto make_callback = [this](const rosbag2_storage::TopicMetadata & topic_data) {
      return [this, topic_data](std::shared_ptr<rclcpp::SerializedMessage> msg) {
               hardware_writer_->write(msg, topic_data.name, topic_data.type, this->now());
             };
    };

  lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    lidar_topic_data.name,
    rclcpp::SensorDataQoS(),
    make_callback(lidar_topic_data)
  );

  hover_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_hover_topic_data.name,
    rclcpp::SensorDataQoS(),
    make_callback(imu_hover_topic_data)
  );

  slamtec_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_slamtec_topic_data.name,
    rclcpp::SensorDataQoS(),
    make_callback(imu_slamtec_topic_data)

  );

  joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
    joints_topic_data.name,
    rclcpp::SystemDefaultsQoS(),
    make_callback(joints_topic_data)
  );
}
