"""
Launch file for the KPIRover simulation UI.

This file launches the following components:
- Gazebo client to display the simulation.
- RViz2 for visualization, using a predefined RViz configuration.
Both components use simulated time.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Define package names and paths.
package_name = 'kpi_rover'
ros_gz_sim = get_package_share_directory('ros_gz_sim')

# Declare and initialize the use_sim_time parameter
use_sim_time = LaunchConfiguration('use_sim_time', default='true')

log_level = LaunchConfiguration('log_level', default='info')

log_level_arg = DeclareLaunchArgument(
    'log_level',
    default_value='info',
    description='Logging level (debug, info, warn, error, fatal)'
)

# Launch the Gazebo client UI
# This node provides the simulation UI for Gazebo.
gzclient_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
    ),
    launch_arguments={
        'gz_args': '-g -v4 ',
        'log_level': log_level,
    }.items()
)

# Launch RViz2 with use_sim_time parameter passed in its configuration
# This node provides a graphical user interface for viewing sensor data and TFs.
rviz = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=[
        '-d', os.path.join(get_package_share_directory(package_name), 'config', 'view_bot.rviz'),
        '--ros-args', '--log-level', log_level,
    ],
    parameters=[{'use_sim_time': use_sim_time}],
    output='screen'
)

# Create and populate the LaunchDescription.
ld = LaunchDescription()
ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'))
ld.add_action(log_level_arg)
ld.add_action(gzclient_cmd)
ld.add_action(rviz)

def generate_launch_description():
    return ld
