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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Define the common clock parameter (not using simulation)
use_sim_time = LaunchConfiguration('use_sim_time', default='false')

# Define parameters for launch_hw
serial_device = LaunchConfiguration('serial_device')
baud_rate = LaunchConfiguration('baud_rate')

package_name = 'kpi_rover'

def generate_launch_description():
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='false', 
        description='Use simulation (Gazebo) clock if true'
    )
    declare_serial_device = DeclareLaunchArgument(
        'serial_device',
        default_value='/dev/ttyAMA2',
        description='Serial device for ECU communication'
    )
    declare_baud_rate = DeclareLaunchArgument(
        'baud_rate',
        default_value='921600',
        description='Baud rate for serial communication'
    )

    # Launch lidar node from cspc_lidar package
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('cspc_lidar'), 'launch', 'lidar.launch.py')
        )
    )

    # Launch ros2_control system for driving real motors
    motors_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', 'launch_hw.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'serial_device': serial_device,
            'baud_rate': baud_rate,
        }.items()
    )

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
        ]
    )

    # Launch the SLAM toolbox for online asynchronous mapping.
    # It builds a map of the environment from sensor data.
    slam_toolbox_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': os.path.join(get_package_share_directory(package_name), 'config', 'slam_toolbox_mapping.yaml'),
            'use_sim_time': use_sim_time
        }.items()
    )

    slam_toolbox_delayed = TimerAction(
        period=2.0,  # Delay to give time for ros2 control to start
        actions=[
            slam_toolbox_map,
        ]
    )

    # Launch the navigation stack.
    # Provides path planning and obstacle avoidance for autonomous robot movement.
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Launch camera node.
    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_node',
        output='screen',
        parameters= [{
            'image_size': [320, 240],
            'video_device':"/dev/video0",
            'output_encoding': "yuv422_yuy2"}],
        remappings=[('/image_raw','/camera/image_raw')]
    )

    # Add all components into the LaunchDescription in the desired sequence.
    return LaunchDescription([
        declare_use_sim_time,
        declare_serial_device,
        declare_baud_rate,
        motors_control,
        ekf,
        lidar,
        slam_toolbox_delayed,
        nav,
        camera
    ])

