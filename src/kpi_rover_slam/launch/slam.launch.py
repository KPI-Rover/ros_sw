import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    slam_params_file = os.path.join(get_package_share_directory("kpi_rover_slam"),
                                   'config', 'slam_toolbox_mapping.yaml')

    start_async_slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("slam_toolbox"), 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': slam_params_file,
                          'use_sim_time': use_sim_time}.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(start_async_slam_toolbox_node)

    return ld
