import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(DeclareLaunchArgument(
        'serial_device',
        default_value='/dev/ttyAMA2',
        description='Serial device for ECU communication'
    ))
    
    declared_arguments.append(DeclareLaunchArgument(
        'baud_rate',
        default_value='921600',
        description='Baud rate for serial communication'
    ))

    declared_arguments.append(DeclareLaunchArgument(
        'encoder_ticks_per_rev',
        default_value='1320',
        description='Number of encoder ticks per revolution'
    ))

    serial_device = LaunchConfiguration('serial_device')
    baud_rate = LaunchConfiguration('baud_rate')
    encoder_ticks_per_rev = LaunchConfiguration('encoder_ticks_per_rev')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Calculate robot description for the controller manager
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare('kpi_rover_description'), 'urdf', 'robot.urdf.xacro']), ' ',
        'use_sim_time:=', use_sim_time, ' ',
        'sim_mode:=false', ' ',
        'serial_device:=', serial_device, ' ',
        'baud_rate:=', baud_rate, ' ',
        'encoder_ticks_per_rev:=', encoder_ticks_per_rev
    ])
    robot_description = {'robot_description': robot_description_content}

    controllers_config = PathJoinSubstitution([
        FindPackageShare('kpi_rover_bringup'),
        'config',
        'kpi_rover_controllers.yaml'
    ])

    # Include the description launch file (starts robot_state_publisher)
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('kpi_rover_description'), 'launch', 'description.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'sim_mode': 'false',
            'serial_device': serial_device,
            'baud_rate': baud_rate,
        }.items()
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            controllers_config,
            {'serial_device': serial_device},
            {'baud_rate': baud_rate}
        ],
        output='screen',
        remappings=[
            ('/diff_drive_controller/odom', '/odom'),
            ('/diff_drive_controller/cmd_vel_unstamped', '/cmd_vel')
        ]
    )

    # Spawners
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen'
    )
    
    imu_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_broadcaster'],
        output='screen'
    )

    delayed_spawners = TimerAction(
        period=3.0,
        actions=[
            joint_state_broadcaster,
            diff_drive_controller,
            imu_broadcaster
        ]
    )

    return LaunchDescription(declared_arguments + [
        description_launch,
        controller_manager,
        delayed_spawners
    ])
