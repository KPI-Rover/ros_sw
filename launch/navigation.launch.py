import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_dir = get_package_share_directory('kpi_rover')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    log_level = LaunchConfiguration('log_level', default='info')

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )

    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    navigation_params_path = os.path.join(
        pkg_dir,
        'config',
        'navigation.yaml'
    )
   
    
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': navigation_params_path,
                'log_level': log_level,
        }.items()
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[('/cmd_vel_in','/cmd_vel_unstamped'),
                       ('/cmd_vel_out','/diff_drive_controller/cmd_vel')]
    )


    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(log_level_arg)
    launchDescriptionObject.add_action(navigation_launch)
    launchDescriptionObject.add_action(twist_stamper)

    return launchDescriptionObject