# Copyright 2026 JustASimpleCoder
#
# Licensed under the Apache License, Version 2.0 (the "License");
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration

from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node

from launch_ros.parameter_descriptions import ParameterValue

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_description_folder = 'paxi_description'

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(robot_description_folder), 'urdf', 'paxi_bot.urdf']
            ),
        ]
    )

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    pkg_share = FindPackageShare(package=robot_description_folder).find(
        robot_description_folder
    )

    default_model_path = os.path.join(pkg_share, 'urdf', 'paxi_bot.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'paxi_bot.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='use_sim_time',
                default_value='False',
                description='Flag to enable use_sim_time',
            ),
            DeclareLaunchArgument(
                name='model',
                default_value=default_model_path,
                description='Absolute path to robot model file',
            ),
            DeclareLaunchArgument(
                name='rvizconfig',
                default_value=default_rviz_config_path,
                description='Absolute path to rviz config file',
            ),
            joint_state_publisher_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
