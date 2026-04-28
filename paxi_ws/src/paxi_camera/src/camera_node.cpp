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

#include "paxi_camera/camera_node.hpp"

namespace paxi_camera
{
CameraNode::CameraNode()
: Node("odom_rgb_node")
{
  this->declare_parameters<std::string>(
    "", {
      {"RGB_camera_info", CameraTopics::OAK_D_LITE_RGB_CAMERA_INFO_TOPIC},
      {"RGB_image_raw", CameraTopics::OAK_D_LITE_RGB_IMAGE_RAW_TOPIC},
      {"RGB_compressed_image", CameraTopics::OAK_D_LITE_RGB_COMPRESSED_TOPIC},
      {"RGB_compressed_depth", CameraTopics::OAK_D_LITE_RGB_COMPRESSED_DEPTH_TOPIC},
      {"RGB_theora_image_raw", CameraTopics::OAK_D_LITE_RGB_THEORA_TOPIC},
      {"stereo_camera_info_raw", CameraTopics::OAK_D_LITE_STEREO_CAMERA_INFO_TOPIC},
      {"stereo_image_raw", CameraTopics::OAK_D_LITE_STEREO_RAW_TOPIC, },
      {"stereo_compressed_image", CameraTopics::OAK_D_LITE_STEREO_COMPRESSED_TOPIC},
      {"stereo_compressed_depth", CameraTopics::OAK_D_LITE_STEREO_COMPRESSED_DEPTH_TOPIC},
      {"stereo_stereo_raw", CameraTopics::OAK_D_LITE_STEREO_THEORA_RAW_TOPIC, }
    });

  std::string rgb_image_raw_topic = this->get_parameter("RGB_image_raw").as_string();
  std::string stereo_image_raw_topic = this->get_parameter("stereo_image_raw").as_string();

  rgb_sub_ = this->create_subscription<ImageMsg>(
    rgb_image_raw_topic,
    rclcpp::SensorDataQoS(),
    [this](const ImageMsg::SharedPtr camera_msg) -> void
    {
      cv_bridge::CvImagePtr cv_ptr =
      cv_bridge::toCvCopy(camera_msg, sensor_msgs::image_encodings::BGR8);
      rgb_msg_queue_.push(cv_ptr->image);
    }
  );

  stereo_sub_ = this->create_subscription<ImageMsg>(
    stereo_image_raw_topic,
    rclcpp::SensorDataQoS(),
    [this](const ImageMsg::SharedPtr camera_msg)-> void
    {
      cv_bridge::CvImagePtr cv_ptr =
      cv_bridge::toCvCopy(camera_msg, sensor_msgs::image_encodings::MONO8);
      stereo_msg_queue_.push(cv_ptr->image);
    }
  );
}

void CameraNode::set_initial_pose_estimate()
{
  // TODO(Jacob): use some known locator to estimate inital pose
  // will need map data
  // process rgb
}

void CameraNode::set_criteria_for_initial_pose_object()
{
  // TODO(Jacob): allow option to set criteria for a new intiali pose object
}

void CameraNode::draw_rectangle_around_inital_pose_object()
{
  // TODO(Jacob): draw rectangles around the criteria that is set
  // have some other way to determine distance to that obect and known position
}

void CameraNode::process_rgb()
{
  while (!rgb_msg_queue_.empty()) {
    cv::Mat curr_image = rgb_msg_queue_.front();
  }
}

}  // namespace paxi_camera
