"""
Launch file for the KPIRover running on real hardware (RPI, lidar, camera, motors with encoders).


This file launches the following components:
- ROS2 control system for motors with joystick control.
- EKF node for sensor fusion and localization.
- SLAM toolbox for online asynchronous mapping.
- Navigation stack for path planning.


"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Define the common clock parameter (not using simulation)
use_sim_time = LaunchConfiguration('use_sim_time', default='false')

log_level = LaunchConfiguration('log_level', default='info')

package_name = 'kpi_rover'

ld = LaunchDescription()
ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'))

log_level_arg = DeclareLaunchArgument(
    'log_level',
    default_value='info',
    description='Logging level (debug, info, warn, error, fatal)'
)

ld.add_action(log_level_arg)

# Launch the EKF node for sensor fusion and localization.
# It fuses sensor data (e.g., IMU, odometry) to estimate the robot's pose.
ekf = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[
        os.path.join(get_package_share_directory(package_name), 'config', 'ekf.yaml'),
        {'use_sim_time': use_sim_time},
    ],
    arguments=['--ros-args', '--log-level', log_level]
)

# Launch the SLAM toolbox for online asynchronous mapping.
# It builds a map of the environment from sensor data.
slam_toolbox_map = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
    ),
    launch_arguments={
        'slam_params_file': os.path.join(get_package_share_directory(package_name), 'config', 'slam_toolbox_mapping.yaml'),
        'use_sim_time': use_sim_time,
        'log_level': log_level,
    }.items()
)

# Launch the navigation stack.
# Provides path planning and obstacle avoidance for autonomous robot movement.
nav = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package_name), 'launch', 'navigation.launch.py')
    ),
    launch_arguments={
        'use_sim_time': use_sim_time,
        'log_level': log_level,
    }.items()
)

# Add all components into the LaunchDescription in the desired sequence.

ld.add_action(ekf)                    # Run EKF for sensor fusion and localization.
ld.add_action(slam_toolbox_map)       # Run SLAM toolkit for mapping.
ld.add_action(nav)                    # Start navigation stack.
def generate_launch_description():
    return ld
