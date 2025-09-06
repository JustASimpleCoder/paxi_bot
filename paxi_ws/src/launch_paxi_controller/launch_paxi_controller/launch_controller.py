import launch

from launch.actions import RegisterEventHandler, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit


from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare




import os

def generate_launch_description():
    
    # path_to_urdf = os.path.join(
    #     os.path.dirname(__file__), "../paxi_ws/launch_paxi_controller/urdf/paxi_bot.urdf"
    # )

    # path_to_controller =  os.path.join(
    #     os.path.dirname(__file__), "../paxi_ws/launch_paxi_controller/controller/paxi_controller.yaml"
    # )
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

    robot_description = {"robot_description" : robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(robot_description_folder),
            "controller",
            "paxi_controller.yaml"
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",        
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],

    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],        
        remappings=[
            ("/hoverboard_base_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hoverboard_base_controller", "--controller-manager", "/controller_manager"],
    )



    delayed_joint_state_broadcaster = TimerAction(
        period=2.0,  # Wait 2 seconds for control_node to be ready
        actions=[joint_state_broadcaster_spawner]
    )
    
    delayed_diff_drive_controller = TimerAction(
        period=4.0,  # Wait 4 seconds to ensure joint_state_broadcaster is loaded first
        actions=[robot_controller_spawner]
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        delayed_joint_state_broadcaster,
        delayed_diff_drive_controller,
    ]

    
    return launch.LaunchDescription(nodes)