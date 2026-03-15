# Copyright 2026 JustASimpleCoder
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os

import pandas as pd

import matplotlib.pyplot as plt

class comparison():
  def __init__(self, filename_1 : str = None, filename_2 : str = None):
    self.imu_slamtec_df = pd.read_csv(filename_1)
    self.odom_controller_df = pd.read_csv(filename_2)


# orientaiton is represented with quaternions
odom_q_names = {
   "q_x": "pose.pose.orientation.x",
   "q_y": "pose.pose.orientation.y",
   "q_z": "pose.pose.orientation.z",
   "q_w": "pose.pose.orientation.w"
}
imu_q_names = {
   "q_x": "orientation.x",
   "q_y": "orientation.y",
   "q_z": "orientation.z",
   "q_w": "orientation.w",
   "g_x": "angular_velocity.x",
   "g_y": "angular_velocity.y",
   "g_z": "angular_velocity.z",
   "a_x": "linear_acceleration.x",
   "a_y": "linear_acceleration.y",
   "a_z": "linear_acceleration.z",
} 

header_sec = "header.stamp.sec"
header_nano_sec = "header.stamp.nanosec"

def main():
  imu_data_flename = "imu_data.csv"
  odometry_controller_filename = "hoverboard_base_controller_odom.csv"

  folder = "~/robotics/paxi_bot_dev/paxi_bot/paxi_ws/sensors_bag_test_ACML_default_3_2026-03-14"

  full_path_imu = os.path.join(folder + "/imu_data", imu_data_flename)
  full_path_odom = os.path.join(folder + "/hoverboard_base_controller_odom", odometry_controller_filename)

  compare = comparison(full_path_imu, full_path_odom)



  print("pd csv 1")
  compare.imu_slamtec_df["total_seconds"] = (compare.imu_slamtec_df[header_sec] + compare.imu_slamtec_df[header_nano_sec] / 1_000_000_000) / 1_000_000_000
  print(compare.imu_slamtec_df[
     [ "total_seconds", imu_q_names["q_x"], imu_q_names["q_y"], imu_q_names["q_z"], imu_q_names["q_w"]]
     ].head(10))


  print("pd csv 2")
  compare.odom_controller_df["total_seconds"] = (compare.odom_controller_df[header_sec] + compare.odom_controller_df[header_nano_sec] / 1_000_000_000) / 1_000_000_000
  
  print(compare.odom_controller_df[
     [ "total_seconds", odom_q_names["q_x"], odom_q_names["q_y"], odom_q_names["q_z"], odom_q_names["q_w"]]
     ].head(10))
  
  print("got here boi")
  
  gyros_stationary_ = compare.imu_slamtec_df.loc[
      (compare.imu_slamtec_df[imu_q_names["g_z"]] < 0.02) & 
      (compare.imu_slamtec_df[imu_q_names["g_z"]] > -0.02), 
      imu_q_names["g_z"]
   ]

  print(gyros_stationary_.describe())

if __name__ == '__main__':
     main()
