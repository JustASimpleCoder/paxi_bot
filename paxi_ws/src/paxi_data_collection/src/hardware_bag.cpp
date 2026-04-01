// Copyright 2026 JustASimpleCoder
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "paxi_data_collection/hardware_bag.hpp"
namespace paxi_data_collection
{
HardwareBag::HardwareBag()
: Node("hardware_bag")
{
  param_listener_ = std::make_shared<ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  hardware_writer_ = std::make_unique<rosbag2_cpp::Writer>();

  hardware_writer_->open(get_bag_name_with_stamp());
  std::vector<std::string> topic_names_ = params_.topic_names;

  num_topics_in_params_ = topic_names_.size();
  start_time_ = this->get_clock()->now();

  topic_endpoint_info_.resize(num_topics_in_params_);
  subscriptions_.reserve(num_topics_in_params_);

  // Need to wait for some ros topics to be discoverable,
  // try again every half second for ~30 secs
  init_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&HardwareBag::init_topic_info, this)
  );

}

const std::string HardwareBag::get_bag_name_with_stamp()
{
  return (params_.stamp_current_date) ? (
    params_.bag_name + "_" + get_current_date()) : params_.bag_name;
}

void HardwareBag::print_topic_added_to_bag(
  const std::string & topic_name, const rclcpp::TopicEndpointInfo & topic_endpoint)
{
  const rclcpp::QoS & topic_qos = topic_endpoint.qos_profile();
  RCLCPP_INFO(
    rclcpp::get_logger("Hardware_BAG"),
    "Added the following topic to the bag:"
    "Topic Name     [%s]\n"
    "Topic Type     [%s]\n"
    "Node Name      [%s]\n"
    "Node Namespace [%s]\n"
    "Endpoint Type   [%d]\n"
    "QoS Profile\n"
    "   History     [%d]\n"
    "   Depth       [%zu]\n"
    "   Reliability [%d]\n"
    "   Durability  [%d]\n",
    topic_name.c_str(), topic_endpoint.topic_type().c_str(), topic_endpoint.node_name().c_str(),
    topic_endpoint.node_namespace().c_str(), static_cast<int>(topic_endpoint.endpoint_type()),
    static_cast<int>(topic_qos.history()), topic_qos.depth(),
    static_cast<int>(topic_qos.reliability()), static_cast<int>(topic_qos.durability()));
}

void HardwareBag::init_topic_info()
{
  if ((this->get_clock()->now() - start_time_).seconds() > MAX_TIME_OUT_SEC) {
    RCLCPP_FATAL(
      rclcpp::get_logger("Data_Collection"),
      "Failed to get all topic info in [%f] seconds", MAX_TIME_OUT_SEC);
    init_timer_->cancel();
    // TO_DO(j): auto delete bag and stop node
    return;
  }

  std::size_t found_all = 0;
  for (std::size_t i = 0; i < num_topics_in_params_; ++i) {

    if (topic_endpoint_info_[i].size() > 0) {
      // already recieved topic info
      ++found_all;
      continue;
    }

    const std::string & topic_name = params_.topic_names[i];
    topic_endpoint_info_[i] = this->get_publishers_info_by_topic(topic_name, false);
    if (topic_endpoint_info_[i].size() <= 0) {
      RCLCPP_INFO(
        rclcpp::get_logger("Data_Collection"),
        "Failed to get topic info for topic name [%s], is it running?", topic_name.c_str());
      continue;
    }
    ++found_all;
  }

  if (found_all == num_topics_in_params_) {
    RCLCPP_INFO(rclcpp::get_logger("Data_Collection"), "Found all Topic information!");
    init_timer_->cancel();
    init_writer_subscribers();
    return;
  }
}

void HardwareBag::init_writer_subscribers()
{

  auto make_callback = [this](const std::string & topic_name, const std::string & topic_msg_type) {
      return [this, topic_name, topic_msg_type](std::shared_ptr<rclcpp::SerializedMessage> msg) {
               hardware_writer_->write(msg, topic_name, topic_msg_type, this->now());
             };
    };

  for (std::size_t i = 0; i < num_topics_in_params_; ++i) {

    // Need index to mach topic name, get_publisher_info_from_topic() returns a vector
    // for all nodes that publish data to that topic
    // hardware writer only writes data based on topic name, message and QoS profile
    // so we just take first element of vector

    const rclcpp::TopicEndpointInfo topic_info = topic_endpoint_info_[i][0];

    rclcpp::QoS qos_profile = topic_info.qos_profile();
    const std::string & topic_msg_type = topic_info.topic_type();
    const std::string & topic_name = params_.topic_names[i];


    hardware_writer_->create_topic(
      {topic_name, topic_msg_type, rmw_get_serialization_format(), ""});

    subscriptions_.emplace_back(
      create_generic_subscription(
        topic_name, topic_msg_type, qos_profile, make_callback(topic_name, topic_msg_type)));

    print_topic_added_to_bag(topic_name, topic_info);
  }

  RCLCPP_INFO(
    rclcpp::get_logger("Data_Collection"),
    "Writer subscribers created and writing to database!");
}

const std::string HardwareBag::get_current_date() const
{
  auto now = std::chrono::system_clock::now();
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  struct tm * parts = std::localtime(&now_c);

  std::stringstream ss;
  ss << std::put_time(parts, "%Y-%m-%d_%H:%M:%S");
  std::string date_str = ss.str();
  return date_str;
}
}  // namespace paxi_data_collection
