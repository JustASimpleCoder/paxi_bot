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

#ifndef PAXI_CAMERA__CAMERA_NODE_HPP_
#define PAXI_CAMERA__CAMERA_NODE_HPP_

#include <cv_bridge/cv_bridge.h>

#include <memory>
#include <queue>
#include <cstdint>

#include "paxi_common/camera_topic_names.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

// #include <image_transport/image_transport.hpp>
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>

namespace paxi_camera
{

namespace CameraTopics = paxi_common::camera_topic_names;

class CameraNode : public rclcpp::Node
{
  using CameraMsgInfo = sensor_msgs::msg::CameraInfo;
  using ImageMsg = sensor_msgs::msg::Image;
  using OdomMsg = nav_msgs::msg::Odometry;

public:
  CameraNode();
  ~CameraNode() = default;

  CameraNode(const CameraNode &) = delete;
  CameraNode(CameraNode &&) = delete;

  const CameraNode & operator=(const CameraNode &) = delete;
  CameraNode && operator=(CameraNode &&) = delete;

  void process_rgb();

  void set_criteria_for_initial_pose_object();
  void draw_rectangle_around_inital_pose_object();

  void set_initial_pose_estimate();

private:
  rclcpp::Subscription<ImageMsg>::ConstSharedPtr rgb_sub_;
  rclcpp::Subscription<ImageMsg>::ConstSharedPtr stereo_sub_;

  std::queue<cv::Mat> rgb_msg_queue_;
  std::queue<cv::Mat> stereo_msg_queue_;
};
}  // namespace paxi_camera

#endif  //  PAXI_CAMERA__CAMERA_NODE_HPP_
