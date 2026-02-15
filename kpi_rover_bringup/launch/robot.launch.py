import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_bringup = get_package_share_directory('kpi_rover_bringup')
    
    # Arguments
    lidar_model_arg = DeclareLaunchArgument(
        'lidar_model',
        default_value='rplidar',
        description='Lidar model to be used'
    )
    
    # Launch HW
    hw_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'hw.launch.py')
        )
    )

    # Launch Lidar
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'lidar.launch.py')
        ),
        launch_arguments={
            'lidar_model': LaunchConfiguration('lidar_model')
        }.items()
    )

    return LaunchDescription([
        lidar_model_arg,
        hw_launch,
        lidar_launch
    ])
