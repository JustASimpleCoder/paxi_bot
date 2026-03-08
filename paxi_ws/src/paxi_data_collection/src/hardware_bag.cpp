#include "paxi_data_collection/hardware_bag.hpp"
namespace paxi_data_collection
{
HardwareBag::HardwareBag()
: Node("hardware_bag")
{

  param_listener_ = std::make_shared<ParamListener>(get_node_parameters_interface());
  hardware_writer_ = std::make_unique<rosbag2_cpp::Writer>();

  params_ = param_listener_->get_params();
  
  std::string bag_name = params_.bag_name;

  if(get_parameter("stamp_current_date").as_bool()){
    bag_name = bag_name + "_" + get_current_date();
  }

  hardware_writer_->open(bag_name);

  std::vector<std::string> topic_names = params_.topic_names;
  std::vector<std::string> topic_msg_types = params_.topic_msg_types;
  std::vector<std::string> topic_qos = params_.topic_msg_qos_types;
  std::size_t num_of_topic = topic_names.size();

  if(num_of_topic != topic_msg_types.size()){
    RCLCPP_FATAL(
      get_logger(),
      "Topic names size [%zu] is not equal to topic_msg_types [%zu]",
      topic_names.size(), topic_msg_types.size()
    );

    throw std::runtime_error("Parameter size mismatch, please ensure topic_names and "
      "topic_msg_types have the same size (and match the correct type by indec)");
  }

  if(num_of_topic != topic_qos.size()){
    RCLCPP_FATAL(
      get_logger(),
      "Topic names size [%zu] is not equal to topic_qos [%zu]",
      topic_names.size(), topic_msg_types.size()
    );

    throw std::runtime_error("Parameter size mismatch, please ensure topic_names and "
      "topic_qos have the same size (and match the correct type by indec)");
  }

  auto make_callback = [this](const std::string & topic_name, const std::string & topic_msg_type) {
    return [this, topic_name, topic_msg_type](std::shared_ptr<rclcpp::SerializedMessage> msg) {
              hardware_writer_->write(msg, topic_name, topic_msg_type, this->now());
            };
  };

  auto get_qos = [](std::string & qos_str)-> rclcpp::QoS {
      if(qos_str == "sensor"){
        return rclcpp::SensorDataQoS();
     
      }
      return rclcpp::SystemDefaultsQoS();
  };

  subscriptions_.reserve(num_of_topic);
  for(std::size_t i = 0; i < num_of_topic; ++i){

    const std::string & topic_name = topic_names[i];
    const std::string & topic_msg_type = topic_msg_types[i];

    hardware_writer_->create_topic(
      {topic_names[i], topic_msg_types[i], rmw_get_serialization_format(), ""});

    subscriptions_.emplace_back(
      create_generic_subscription(
        topic_name, topic_msg_type,
        get_qos(topic_names[i]),
        make_callback(topic_name, topic_msg_type)
      )
    );
  }
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
}  // namespace paxi_data_collection