import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
   
    # Package paths
    package_name = 'kpi_rover'
    pkg_share = get_package_share_directory(package_name)

    # Convert camera format for viewing in rviz
    ld.add_action(Node(
        package='kpi_rover',
        executable='yuv422_compressed_to_rgb8_converter.py',
        name='yuv422_compressed_to_rgb8_converter',
        output='screen',
        remappings=[('/in','/image_raw/compressed'),
                    ('/out','/image_rgb8')]
    ))

    # RViz configuration file
    rviz_config_file = os.path.join(pkg_share, 'config', 'view_bot.rviz')


    # Start RViz2
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    ))

    return ld
