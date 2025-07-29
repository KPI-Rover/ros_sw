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

# Launch the Gazebo client UI
# This node provides the simulation UI for Gazebo.
gzclient_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
    ),
    launch_arguments={'gz_args': '-g -v4 '}.items()
)

# Launch RViz2 with use_sim_time parameter passed in its configuration
# This node provides a graphical user interface for viewing sensor data and TFs.
rviz = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', os.path.join(get_package_share_directory(package_name), 'config', 'view_bot.rviz')],
    parameters=[{'use_sim_time': use_sim_time}],
    output='screen'
)

stereo_image_view = Node(
    package='image_view',
    executable='stereo_view',
    name='stereo_image_view',
    output='screen',
    parameters=[{
        'stereo':'/stereo_camera',
        'image': 'image_rect_color',
        'use_sim_time': use_sim_time
        },
    ]
)

# Create and populate the LaunchDescription.
ld = LaunchDescription()
ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'))
ld.add_action(gzclient_cmd)
ld.add_action(rviz)
ld.add_action(stereo_image_view)

def generate_launch_description():
    return ld
