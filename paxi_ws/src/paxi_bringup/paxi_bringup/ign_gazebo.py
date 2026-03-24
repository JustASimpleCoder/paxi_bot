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
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction

from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = FindPackageShare('paxi_description')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([pkg, 'urdf', 'paxi_bot_simulation.urdf']),
        ]
    )

    # Launch Gazebo with office world
    gz_sim = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
        ),
        launch_arguments={
            'gz_args': [PathJoinSubstitution([pkg, 'world', 's_office.sdf']), ' -r']
        }.items(),
    )

    # Publish robot description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description_content,
                'use_sim_time': True,
            }
        ],
        output='screen',
    )

    # Spawn robot into Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',
            'paxi_bot',
            '-topic',
            '/robot_description',
            '-world',
            'office_world',
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.096',
        ],
        output='screen',
    )

    # ROS-Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Controllers
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
    )

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'hoverboard_base_controller',
            '--controller-manager',
            '/controller_manager',
        ],
        remappings=[
            ('/hoverboard_base_controller/cmd_vel_unstamped', '/cmd_vel'),
        ],
    )

    return LaunchDescription(
        [
            gz_sim,
            robot_state_publisher,
            TimerAction(period=5.0, actions=[spawn_robot]),
            TimerAction(period=6.0, actions=[bridge]),
            TimerAction(period=15.0, actions=[joint_state_broadcaster]),
            TimerAction(period=17.0, actions=[diff_drive_controller]),
        ]
    )
