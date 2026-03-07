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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_share = get_package_share_directory("paxi_description")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(pkg_share, "maps", "feb_26_room_hallway.yaml"),
        description="Full path to the map yaml file to use for navigation",
    )

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_share, "config", "nav2_params.yaml"),
        description="Full path to the nav2 params yaml file",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
        }.items(),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            map_arg,
            params_arg,
            nav2_bringup,
        ]
    )
