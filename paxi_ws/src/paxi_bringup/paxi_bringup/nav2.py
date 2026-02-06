# Copyright 2019 Open Source Robotics Foundation, Inc.
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
#
# Author: Darby Lim
#
# Modified November 26, 2025 by Jacob Cohen


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def get_ros_sys_path(foldername, filename, package_name):
    return PathJoinSubstitution([FindPackageShare(package_name), foldername, filename])


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    map_dir = LaunchConfiguration(
        "map",
        default=os.path.join(
            get_package_share_directory("paxi_description"),
            "maps",
            "my_room_and_hallway_feb3.yaml",
        ),
    )

    nav2_param_file_name = "nav2_params.yaml"

    param_dir = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            get_package_share_directory("paxi_description"),
            "config",
            nav2_param_file_name,
        ),
    )

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )

    rviz_config_dir = os.path.join(
        get_package_share_directory("paxi_description"), "rviz", "nav2.rviz"
    )

    nav2_map_Server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_ros_sys_path("launch", "navigation_launch.py", "nav2_map_Server")]
        ),
        launch_arguments={
            "use_sim_time": "false",
            "map": map_dir,
            "use_sim_time": use_sim_time,
            "params_file": param_dir,
        }.items(),
    )

    activate_map_server = TimerAction(
        period=3.0,  # Wait 3 seconds after launch
        actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/map_server", "activate"],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=map_dir,
                description="Full path to map file to load",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=param_dir,
                description="Full path to param file to load",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [nav2_launch_file_dir, "/bringup_launch.py"]
                ),
                launch_arguments={
                    "map": map_dir,
                    "use_sim_time": use_sim_time,
                    "params_file": param_dir,
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_dir],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            # activate_map_server,
        ]
    )
