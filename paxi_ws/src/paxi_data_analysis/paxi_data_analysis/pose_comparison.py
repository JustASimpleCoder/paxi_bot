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

import numpy

import pandas as pd


# orientaiton is represented with quaternions
odom_q_names = {
    'q_x': 'pose.pose.orientation.x',
    'q_y': 'pose.pose.orientation.y',
    'q_z': 'pose.pose.orientation.z',
    'q_w': 'pose.pose.orientation.w',
    'ang_z': 'twist.twist.angular.z',
}
imu_q_names = {
    'q_x': 'orientation.x',
    'q_y': 'orientation.y',
    'q_z': 'orientation.z',
    'q_w': 'orientation.w',
    'g_x': 'angular_velocity.x',
    'g_y': 'angular_velocity.y',
    'g_z': 'angular_velocity.z',
    'a_x': 'linear_acceleration.x',
    'a_y': 'linear_acceleration.y',
    'a_z': 'linear_acceleration.z',
}

header_sec = 'header.stamp.sec'
header_nano_sec = 'header.stamp.nanosec'


class DataHandler:

    def __init__(self, filename: str = None, dict_col_names: dict = None):
        self.df = pd.read_csv(filename)
        self.col_names = dict_col_names

    def get_quaternion_data(self) -> pd:
        return self.df[
            [
                header_sec,
                header_nano_sec,
                self.col_names['q_x'],
                self.col_names['q_y'],
                self.col_names['q_z'],
                self.col_names['q_w'],
            ]
        ]

    def get_yaw_fram_quaternion(self, to_degree: bool = False) -> pd:

        q_x = self.df[self.col_names['q_x']]
        q_y = self.df[self.col_names['q_y']]
        q_z = self.df[self.col_names['q_z']]
        q_w = self.df[self.col_names['q_w']]

        # yaw = atan2(2⋅(q4⋅q3+q1⋅q2),1−2⋅(q2^2+q3^2))
        yaw = 2 * numpy.arctan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y * q_y + q_z * q_z))

        yaw = yaw if not to_degree else yaw * (180 / numpy.pi)

        return pd.DataFrame(
            {
                header_sec: self.df[header_sec],
                header_nano_sec: self.df[header_nano_sec],
                'yaw': yaw,
                self.col_names['g_z']: self.df[self.col_names['g_z']],
            }
        )

    def get_average_in_ros_time_sec(self, time: int, df: pd = None) -> pd:
        if df is None:
            return self.df[self.df[header_sec] == time]
        else:
            return df[df[header_sec] == time]


class comparison:

    def __init__(self, filename_1: str = None, filename_2: str = None):
        self.imu_slamtec_df = pd.read_csv(filename_1)
        self.odom_controller_df = pd.read_csv(filename_2)


def main():
    imu_data_flename = 'imu_data.csv'
    odometry_controller_filename = 'hoverboard_base_controller_odom.csv'

    folder = '~/robotics/paxi_bot_dev/paxi_bot/paxi_ws/sensors_bag_test_ACML_default_3_2026-03-14'

    full_path_imu = os.path.join(folder + '/imu_data', imu_data_flename)
    full_path_odom = os.path.join(
        folder + '/hoverboard_base_controller_odom', odometry_controller_filename
    )

    imu_data = DataHandler(full_path_imu, imu_q_names)
    odom_data = DataHandler(full_path_odom, odom_q_names)

    quats = imu_data.get_quaternion_data()
    yaws_rad = imu_data.get_yaw_fram_quaternion()
    yaws_degree = imu_data.get_yaw_fram_quaternion(True)
    print(quats)
    print(yaws_rad)
    print(yaws_degree)

    time = 1773519838

    odom_data_in_time_frame = odom_data.get_average_in_ros_time_sec(
        time, odom_data.df[[header_sec, header_nano_sec, odom_data.col_names['ang_z']]]
    )
    yaws_rad_in_time_frame = imu_data.get_average_in_ros_time_sec(time, yaws_rad)

    print(odom_data_in_time_frame)
    print(yaws_rad_in_time_frame)

    v_yaws_rad_in_time_frame = pd.DataFrame()
    v_yaws_rad_in_time_frame['d_time'] = yaws_rad_in_time_frame[header_nano_sec].diff()
    v_yaws_rad_in_time_frame['d_yaw'] = yaws_rad_in_time_frame['yaw'].diff()
    v_yaws_rad_in_time_frame['gyro_z'] = yaws_rad_in_time_frame[imu_q_names['g_z']]
    v_yaws_rad_in_time_frame['v_yaw'] = v_yaws_rad_in_time_frame['d_yaw'] / (
        v_yaws_rad_in_time_frame['d_time'] / 1_000_000_000
    )  # - (9.8*sin(2*pi/180))

    print(v_yaws_rad_in_time_frame)

    describe_odom = odom_data_in_time_frame[odom_data.col_names['ang_z']].describe()
    describe_imu = v_yaws_rad_in_time_frame[v_yaws_rad_in_time_frame['v_yaw'] > 0.22][
        'v_yaw'
    ].describe()
    describe_gyro_imu = v_yaws_rad_in_time_frame['gyro_z'].describe()

    mean_odom = odom_data_in_time_frame[odom_data.col_names['ang_z']].mean()
    mean_imu = v_yaws_rad_in_time_frame[v_yaws_rad_in_time_frame['v_yaw'] > 0.22][
        'v_yaw'
    ].mean()
    mean_gyro_imu = v_yaws_rad_in_time_frame['gyro_z'].mean()

    print(f'stats meand odom:   {describe_odom}')
    print(f'stats imu vyaw:     {describe_imu}')
    print(f'stats imu gyro_yaw:     {describe_gyro_imu}')

    print(f'odom mean {mean_odom}')
    print(f'mean_imu {mean_imu}')
    print(f'mean_gyro_imu {mean_gyro_imu}')

    print(f'Differences vyaw {mean_odom - mean_imu}')
    print(f'Differences gyro {mean_odom - mean_gyro_imu}')

    # all data
    mean_orig_odom_data = odom_data.df[odom_data.col_names['ang_z']].mean()
    mean_orig_gyro_data = imu_data.df[imu_data.col_names['g_z']].mean()

    print(f'mean origin odom        {mean_orig_odom_data}')
    print(f'mean imu gyro data     {mean_orig_gyro_data}')


if __name__ == '__main__':
    main()
