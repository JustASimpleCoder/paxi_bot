

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    robot_description_folder = "paxi_description"

    pkg_share = FindPackageShare(package=robot_description_folder).find(robot_description_folder)

    default_model_path = os.path.join(pkg_share, 'urdf', 'paxi_bot.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'paxi_bot.rviz')
    world_path=os.path.join(pkg_share, 'world' ,'my_world.sdf')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, {'use_sim_time':LaunchConfiguration('use_sim_time')}]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        # name='joint_state_publisher',
        # parameters=[{'robot_description': Command(['xacro ', default_model_path])}],
        #condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name = 'ekf_node',
        output='screen',
        parameters = [os.path.join(pkg_share,'config/nav2_ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    
    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='False', description= 'Flag to enable use_sim_time'),
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        #ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        robot_localization_node,
    ])