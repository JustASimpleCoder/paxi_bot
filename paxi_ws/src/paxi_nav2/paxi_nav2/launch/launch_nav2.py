import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package Directories
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')
    pkg_paxi_description = FindPackageShare('paxi_description')
    pkg_paxi_navigation = FindPackageShare('paxi_navigation')  # Your navigation package
    
    # Paths to folders and files
    nav2_params_file = PathJoinSubstitution([
        pkg_paxi_navigation,
        'config',
        'nav2_params.yaml'
    ])
    
    map_file = PathJoinSubstitution([
        pkg_paxi_navigation,
        'maps',
        'map.yaml'
    ])
    
    robot_urdf = PathJoinSubstitution([
        pkg_paxi_description,
        'urdf',
        'paxi_bot.urdf.xacro'
    ])
    
    rviz_config_file = PathJoinSubstitution([
        pkg_paxi_navigation,
        'rviz',
        'nav2_config.rviz'
    ])

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    map_yaml = LaunchConfiguration('map')

    # Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', robot_urdf])
        }]
    )

    # Nav2 bringup launch
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_nav2_bringup,
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
        }.items(),
    )

    # RViz
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(DeclareLaunchArgument(
        'map',
        default_value=map_file,
        description='Full path to map yaml file to load'))

    ld.add_action(DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to param file to load'))

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'))

    ld.add_action(DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack'))

    ld.add_action(DeclareLaunchArgument(
        'use_composition', 
        default_value='True',
        description='Whether to use composed bringup'))

    ld.add_action(DeclareLaunchArgument(
        'use_respawn', 
        default_value='False',
        description='Whether to respawn if a node crashes'))

    # Add the commands to the launch description
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(rviz_cmd)

    return ld