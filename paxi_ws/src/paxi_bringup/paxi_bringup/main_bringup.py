# Copyright 2026 JustASimpleCoder
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import launch

from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction

from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


robot_description_folder = 'paxi_description'


def get_sys_path(foldername, filename: str):
    return PathJoinSubstitution(
        [FindPackageShare(robot_description_folder), foldername, filename]
    )


def generate_launch_description():

    # robot_description_folder = 'paxi_description'

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            get_sys_path('urdf', 'paxi_bot.urdf'),
        ]
    )

    controller_filename = LaunchConfiguration(
        'controller_config_filename', default='paxi_controller.yaml'
    )
    controller_filename_arg = DeclareLaunchArgument(
        'controller_config_filename',
        description='path to diff drive controller config (YAML file) '
        'holding controller parameters',
        default_value='paxi_controller.yaml',
    )

    robot_description = {'robot_description': robot_description_content}
    robot_controller_config = get_sys_path('controller', controller_filename)

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controller_config],
        output='both',
        remappings=[
            ('~/robot_description', '/robot_description'),
            # ('/hoverboard_base_controller/cmd_vel_unstamped', '/cmd_vel'),
        ],
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'hoverboard_base_controller',
            '--controller-manager',
            '/controller_manager',
        ],
    )

    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/rplidar')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='base_scan')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')

    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[
            {
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode,
                'use_sim_time': False,
            }
        ],
        output='screen',
    )

    # serial_port = LaunchConfiguration('serial_port', default='/dev/rplidar')
    slamkit_node_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare('slamkit_ros2'), 'launch', 'slamkit_usb.py']
        ),
    )

    # complementary_filter_node = Node(
    #     package='imu_complementary_filter',
    #     executable='complementary_filter_node',
    #     name='complementary_filter_node',
    #     output='screen',
    #     parameters=[
    #         {
    #             'frame_id': 'imu_slamtec',
    #             'publish_debug_topics': False,
    #             'gain_acc': 0.01,
    #         }
    #     ],
    # )

    madgwick_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_node',
        output='screen',
        parameters=[
            {
                'use_mag': True,
                'gain': 0.3,
                # for zeta data sheet says
                # zero initialization 5 deg/s -> 0.08726646259 rad/s,
                # my own stats calc -0.0104584
                'zeta': 0.001,
                # 'gain_acc': 0.01,
                'world_frame': 'enu',
                'orientation_stddev': 0.001,
                'publish_tf': False,
                # 'fixed_frame': 'base_link',
                'use_sim_time': False,
            },
        ],
    )

    efk_filename = LaunchConfiguration('ekf_filename', default='nav2_ekf.yaml')
    efk_filename_arg = DeclareLaunchArgument(
        'ekf_filename',
        description='path to robot localization YAML file holding ekf parameters',
        default_value='nav2_ekf.yaml',
    )

    ekf_config_path = get_sys_path('config', efk_filename)

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': False}],
    )

    delayed_joint_state_broadcaster = TimerAction(
        period=2.0,  # Wait 2 seconds for control_node to be ready
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_diff_drive_controller = TimerAction(
        period=4.0,  # Wait 4 seconds to ensure joint_state_broadcaster is loaded first
        actions=[robot_controller_spawner],
    )

    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_to_hoverboard_relay',
        arguments=[
            '/cmd_vel',
            '/hoverboard_base_controller/cmd_vel_unstamped',
            'geometry_msgs/msg/Twist',
        ],
        output='screen',
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        delayed_joint_state_broadcaster,
        delayed_diff_drive_controller,
        lidar_node,
        slamkit_node_launch,
        # complementary_filter_node,
        madgwick_node,
        efk_filename_arg,
        controller_filename_arg,
        robot_localization_node,
        cmd_vel_relay,
    ]

    return launch.LaunchDescription(nodes)
