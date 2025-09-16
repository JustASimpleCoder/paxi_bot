

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
import os

def generate_launch_description():

    robot_description_folder = "paxi_description"

    robot_description_content = Command(
        [
            PathJoinSubstitution( [FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(robot_description_folder),
                    "urdf",
                    "paxi_bot.urdf"
                ]
            )
        ]
    )

    robot_description = {"robot_description" :ParameterValue(robot_description_content, value_type=str)}

    pkg_share = FindPackageShare(package=robot_description_folder).find(robot_description_folder)

    default_model_path = os.path.join(pkg_share, 'urdf', 'paxi_bot.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'paxi_bot.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],        
#       remappings=[
#           ("/hoverboard_base_controller/cmd_vel_unstamped", "/cmd_vel"),
#       ],
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

    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='base_scan')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')


    #TODO: only launch lidar_node if lidar is connected
    lidar_node =   Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type':channel_type, 
            'serial_port': serial_port, 
            'serial_baudrate': serial_baudrate, 
            'frame_id': frame_id,
            'inverted': inverted, 
            'angle_compensate': angle_compensate,
            'scan_mode': scan_mode}],
        output='screen',
    )



    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='False', description= 'Flag to enable use_sim_time'),
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        lidar_node,
    ])