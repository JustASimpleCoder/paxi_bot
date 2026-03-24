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

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration

from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # pkg_share = get_package_share_directory('paxi_description')
    data_collection_filename = LaunchConfiguration(
        'params_file', default='paxi_data_collection.yaml'
    )

    data_collection_filename_arg = DeclareLaunchArgument(
        'params_file',
        default_value='paxi_data_collection.yaml',
        description='File name to the data collection params yaml file',
    )

    data_collection_params_path = PathJoinSubstitution(
        [
            FindPackageShare('paxi_description'),
            'data_collection',
            data_collection_filename,
        ]
    )

    data_collection_node = Node(
        package='paxi_data_collection',
        executable='data_collection',
        name='hardware_bag',
        parameters=[data_collection_params_path],
        output='screen',
    )

    return LaunchDescription([data_collection_filename_arg, data_collection_node])
