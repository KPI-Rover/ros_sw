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

    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.034',
        description='Wheel radius in meters'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Launch HW
    hw_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'hw.launch.py')
        ),
        launch_arguments={
            'wheel_radius': LaunchConfiguration('wheel_radius')
        }.items()
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

    # Launch SLAM
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('kpi_rover_slam'), 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    return LaunchDescription([
        lidar_model_arg,
        wheel_radius_arg,
        use_sim_time_arg,
        hw_launch,
        lidar_launch,
        slam_launch
    ])
