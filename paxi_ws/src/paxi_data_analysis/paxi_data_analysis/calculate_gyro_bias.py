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

    # TODO(Jacob): standardize this so we dont pass full path -> on main branch will be located differently
    # Just ened to figure out where we want to place bags from sensor bags and using ros2-unbag
    # where to place .csv files
    imu_data_flename = "imu_data.csv"
    folder = "~/robotics/paxi_bot_dev/paxi_bot/paxi_ws/sensors_bag_test_ACML_default_3_2026-03-14"
    full_path_imu = os.path.join(folder + "/imu_data", imu_data_flename)

    imu_slamtec_df = pd.read_csv(full_path_imu)

    gyros_stationary_ = imu_slamtec_df.loc[
        (imu_slamtec_df[imu_q_names["g_z"]] < 0.01)
        & (imu_slamtec_df[imu_q_names["g_z"]] > -0.02),
        imu_q_names["g_z"],
    ]
    
    print(gyros_stationary_.describe())
    print(f"Mean is [{gyros_stationary_.mean()}]")


if __name__ == "__main__":
    main()
