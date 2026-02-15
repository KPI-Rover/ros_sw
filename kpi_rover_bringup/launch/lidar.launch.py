import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    lidar_model = LaunchConfiguration('lidar_model')

    declare_lidar_model_cmd = DeclareLaunchArgument(
        'lidar_model',
        default_value='rplidar',
        description='Lidar model to be used. Supported models: rplidar, coin'
    )

    # Launch lidar node from cspc_lidar package
    lidar_coin_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('cspc_lidar'), 'launch', 'lidar.launch.py')
        ),
        condition=IfCondition(PythonExpression(["'", lidar_model, "' == 'coin'"]))
    )

    lidar_rp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_c1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/lidar',
            'serial_baudrate': '460800',
            'frame_id': 'laser_frame',
        }.items(),
        condition=IfCondition(PythonExpression(["'", lidar_model, "' == 'rplidar'"]))
    )

    return LaunchDescription([
        declare_lidar_model_cmd,
        lidar_coin_launch,
        lidar_rp_launch
    ])
