   

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  PathJoinSubstitution
#from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


robot_description_folder = "paxi_description"

def get_ros_sys_path(foldername, filename, package_name):
    return PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            foldername,
            filename
        ]
    )



def get_sys_path(foldername, filename):
    return PathJoinSubstitution(
        [
            FindPackageShare(robot_description_folder),
            foldername,
            filename
        ]
    )


def generate_launch_description():

    slam_params_file = get_sys_path('config', 'slam_params.yaml')

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_ros_sys_path('launch', 'online_async_launch.py', 'slam_toolbox')
        ]),
        launch_arguments={'use_sim_time': 'false',
                          'params_file': slam_params_file
                          }.items()
    )


    nav2_async_live_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_ros_sys_path('launch', 'navigation_launch.py', 'nav2_bringup')
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    #ros2 run nav2_costmap_2d nav2_costmap_2d_markers 
    #voxel_grid:=/local_costmap/voxel_grid visualization_marker:=/my_marker
    nav2_add_voxel_marker_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d_markers',
        name='voxel_markers',
        output='screen',
        remappings=[
            ('voxel_grid', '/local_costmap/voxel_grid'), 
            ('visualization_marker', '/my_marker')
        ]
    )

    nodes = [
        slam_toolbox_launch,
        nav2_async_live_mapping_launch,
        #nav2_add_voxel_marker_node
    ]

    return launch.LaunchDescription(nodes)