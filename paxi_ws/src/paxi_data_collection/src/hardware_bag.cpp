#include "paxi_data_collection/hardware_bag.hpp"

HardwareBag::HardwareBag()
: Node("hardware_bag")
{
  hardware_writer_ = std::make_unique<rosbag2_cpp::Writer>();
  

  this->declare_parameter("bag_name","sensors_bag");
  this->declare_parameter("stamp_current_date", true);

  std::string bag_name = get_parameter("bag_name").as_string();

  if(get_parameter("stamp_current_date").as_bool()){
    bag_name = bag_name + "_" + get_current_date();
  }

  hardware_writer_->open(bag_name);

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

  const rosbag2_storage::TopicMetadata odom_filtered_topic_data = {
    "odometry/filtered",
    "sensor_msgs/msg/JointState",
    rmw_get_serialization_format(),
    ""
  };

  const rosbag2_storage::TopicMetadata accel_filtered_topic_data = {
    "accel/filtered",
    "sensor_msgs/msg/JointState",
    rmw_get_serialization_format(),
    ""
  };

  const rosbag2_storage::TopicMetadata ekf_tf_topic_data = {
    "/tf",
    "sensor_msgs/msg/JointState",
    rmw_get_serialization_format(),
    ""
  };

  hardware_writer_->create_topic(lidar_topic_data);
  hardware_writer_->create_topic(imu_hover_topic_data);
  hardware_writer_->create_topic(imu_slamtec_topic_data);
  hardware_writer_->create_topic(joints_topic_data);
  hardware_writer_->create_topic(odom_filtered_topic_data);
  hardware_writer_->create_topic(accel_filtered_topic_data);
  hardware_writer_->create_topic(ekf_tf_topic_data);


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


const std::string HardwareBag::get_current_date() const
{
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    struct tm* parts = std::localtime(&now_c);

    std::stringstream ss;
    ss << std::put_time(parts, "%Y-%m-%d");
    std::string date_str = ss.str();
    return date_str;
}