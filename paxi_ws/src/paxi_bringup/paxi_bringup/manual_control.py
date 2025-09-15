from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="teleop_twist_keyboard",
            executable="teleop_twist_keyboard",
            name="teleop",
            output="screen",
            prefix="gnome-terminal --",   # opens in its own terminal so you can type
            remappings=[
                ("/hoverboard_base_controller/cmd_vel_unstamped", "/cmd_vel"),  # change RHS if your robot uses a different topic
            ]
        )
    ])