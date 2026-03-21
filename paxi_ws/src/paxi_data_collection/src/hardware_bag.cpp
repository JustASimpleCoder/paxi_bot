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

  if (params_.stamp_current_date) {
    bag_name = bag_name + "_" + get_current_date();
  }

  hardware_writer_->open(bag_name);

  std::vector<std::string> topic_names = params_.topic_names;
  std::size_t num_of_topic = topic_names.size();

  auto make_callback = [this](const std::string & topic_name, const std::string & topic_msg_type) {
      return [this, topic_name, topic_msg_type](std::shared_ptr<rclcpp::SerializedMessage> msg) {
               hardware_writer_->write(msg, topic_name, topic_msg_type, this->now());
             };
    };

  subscriptions_.reserve(num_of_topic);

  for (std::size_t i = 0; i < num_of_topic; ++i) {

    const std::string & topic_name = topic_names[i];
    publisher_info_ = this->get_publishers_info_by_topic(topic_name, false);

    if(publisher_info_.size() <= 0){
      RCLCPP_INFO(rclcpp::get_logger("Data_Collection"),
      "Failed to get topic info for topic name [%s], is it running?", topic_name.c_str());
      continue;
    }

    
    rclcpp::QoS qos_profile = publisher_info_[0].qos_profile();
    const std::string & topic_msg_type = publisher_info_[0].topic_type();

    publisher_info_[0].node_name();

    hardware_writer_->create_topic(
      {topic_names[i], topic_msg_type, rmw_get_serialization_format(), ""});

    subscriptions_.emplace_back(
      create_generic_subscription(
        topic_name, topic_msg_type,
        qos_profile,
        make_callback(topic_name, topic_msg_type)
      )
    );

    print_topic_added_to_bag(topic_name, publisher_info_[0]);
  }
}

void HardwareBag::print_topic_added_to_bag(const std::string & topic_name, const rclcpp::TopicEndpointInfo &  topic_endpoint)
{

  const rclcpp::QoS & topic_qos = topic_endpoint.qos_profile();
  RCLCPP_INFO(rclcpp::get_logger("Hardware_BAG"),
      "Added the following topic to the bag:"\
      "Topic Name     [%s]\n"\
      "Topic Type     [%s]\n"\
      "Node Name      [%s]\n"\
      "Node Namespace [%s]\n"\
      "Endpoint Type   [%d]\n"\
      "QoS Profile\n"\
      "   History     [%d]\n"\
      "   Depth       [%zu]\n"\
      "   Reliability [%d]\n"\
      "   Durability  [%d]\n",
      topic_name.c_str(),
      topic_endpoint.topic_type().c_str(),
      topic_endpoint.node_name().c_str(),
      topic_endpoint.node_namespace().c_str(),
      static_cast<int>(topic_endpoint.endpoint_type()),
      static_cast<int>(topic_qos.history()),
      topic_qos.depth(),
      static_cast<int>(topic_qos.reliability()),
      static_cast<int>(topic_qos.durability())
  );
}

const std::string HardwareBag::get_current_date() const
{
  auto now = std::chrono::system_clock::now();
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  struct tm * parts = std::localtime(&now_c);

  std::stringstream ss;
  ss << std::put_time(parts, "%Y-%m-%d");
  std::string date_str = ss.str();
  return date_str;
}
}  // namespace paxi_data_collection
