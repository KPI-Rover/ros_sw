import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_mode = LaunchConfiguration('sim_mode')
    serial_device = LaunchConfiguration('serial_device')
    baud_rate = LaunchConfiguration('baud_rate')

    pkg_share = FindPackageShare('kpi_rover_description')
    
    # Process the URDF file
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])
    
    robot_description_config = Command([
        'xacro ', xacro_file, 
        ' use_sim_time:=', use_sim_time, 
        ' sim_mode:=', sim_mode,
        ' serial_device:=', serial_device,
        ' baud_rate:=', baud_rate
    ])

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'sim_mode',
            default_value='false',
            description='Enable simulation mode'),
        DeclareLaunchArgument(
            'serial_device',
            default_value='/dev/ttyAMA2',
            description='Serial device for ECU communication'),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='921600',
            description='Baud rate for serial communication'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_config, 'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])
