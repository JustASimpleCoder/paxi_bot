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

#ifndef PAXI_COMMON__VISUAL_ODOM_TOPIC_NAMES_HPP_
#define PAXI_COMMON__VISUAL_ODOM_TOPIC_NAMES_HPP_

namespace paxi_common::visual_odom_topics
{

  // camera info topic name from OAK_D_LIT ros2 brdige
  inline constexpr const char * OAK_D_LITE_RGB_CAMERA_INFO_TOPIC = "/driver/rgb/camera_info";

  // RGB image raw topic name from OAK_D_LIT ros2 brdige
  inline constexpr const char * OAK_D_LITE_RGB_IMAGE_RAW_TOPIC = "/driver/rgb/image_raw";

  // RGB Compressed image raw topic name from OAK_D_LIT ros2 brdige
  inline constexpr const char * OAK_D_LITE_RGB_COMPRESSED_TOPIC = "/driver/rgb/image_raw/compressed";

  // RGB Compressed Depth topic  topic name from OAK_D_LIT ros2 brdige
  inline constexpr const char * OAK_D_LITE_RGB_COMPRESSED_DEPTH_TOPIC ="/driver/rgb/image_raw/compressedDepth";

  // RBG THEORA image raw topic name from OAK_D_LIT ros2 brdige
  inline constexpr const char * OAK_D_LITE_RGB_THEORA_TOPIC = "/driver/rgb/image_raw/theora ";

  // Stero image raw topic name from OAK_D_LIT ros2 brdige
  inline constexpr const char * OAK_D_LITE_STEREO_CAMER_INFO_TOPIC ="/driver/stereo/camera_info ";

  // Stereo image raw topic name from OAK_D_LIT ros2 brdige
  inline constexpr const char * OAK_D_LITE_STEREO_RAW_TOPIC = "/driver/stereo/image_raw ";

  // Stereo  name from OAK_D_LIT ros2 brdige
  inline constexpr const char * OAK_D_LITE_STEREO_COMPRESSED_TOPIC = "/driver/stereo/image_raw/compressed";

  // Stereo info topic name from OAK_D_LIT ros2 brdige
  inline constexpr const char * OAK_D_LITE_STEREO_COMPRESSED_DEPTH_TOPIC = "/driver/stereo/image_raw/compressedDepth";

  // Stereo THEORA info topic name from OAK_D_LIT ros2 brdige
  inline constexpr const char * OAK_D_LITE_STEREO_THEORA_RAW_TOPIC = "/driver/stereo/image_raw/theora";

}  // namespace paxi_common::hardware_topics

#endif  // PAXI_COMMON__HARDWARE_TOPIC_NAMES_HPP_
