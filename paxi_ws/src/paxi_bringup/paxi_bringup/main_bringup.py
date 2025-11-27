import launch
from launch.actions import RegisterEventHandler, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


robot_description_folder = "paxi_description"

def get_sys_path(foldername, filename):
    return PathJoinSubstitution(
        [
            FindPackageShare(robot_description_folder),
            foldername,
            filename
        ]
    )

def generate_launch_description():
    
    #robot_description_folder = "paxi_description"

    robot_description_content = Command(
        [
            PathJoinSubstitution( [FindExecutable(name="xacro")]),
            " ",
            get_sys_path("urdf", "paxi_bot.urdf")
        ]
    )

    robot_description = {"robot_description" : robot_description_content}
    robot_controller_config = get_sys_path("controller","paxi_controller.yaml") 
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controller_config],
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

    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB1')
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

    ekf_config_path = get_sys_path("config", "nav2_ekf.yaml")

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name = 'ekf_node',
        output='screen',
        parameters = [ekf_config_path, {'use_sim_time': False}]
    )

    delayed_joint_state_broadcaster = TimerAction(
        period=2.0,  # Wait 2 seconds for control_node to be ready
        actions=[joint_state_broadcaster_spawner]
    )
    
    delayed_diff_drive_controller = TimerAction(
        period=4.0,  # Wait 4 seconds to ensure joint_state_broadcaster is loaded first
        actions=[robot_controller_spawner]
    )

    # cmd_vel_relay = Node(
    #     package='topic_tools',
    #     executable='relay',
    #     name='cmd_vel_to_hoverboard_relay',
    #     arguments=['/cmd_vel', '/hoverboard_base_controller/cmd_vel_unstamped', 'geometry_msgs/msg/Twist'],
    #     output='screen'
    # )

    nodes = [
        control_node,
        robot_state_pub_node,
        delayed_joint_state_broadcaster,
        delayed_diff_drive_controller,
        lidar_node,
        robot_localization_node,
        #cmd_vel_relay
    ]

    
    return launch.LaunchDescription(nodes)