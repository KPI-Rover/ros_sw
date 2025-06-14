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
ecu_ip = LaunchConfiguration('ecu_ip')
ecu_port = LaunchConfiguration('ecu_port')
udp_port = LaunchConfiguration('udp_port')

log_level = LaunchConfiguration('log_level', default='info')

package_name = 'kpi_rover'

ld = LaunchDescription()
ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'))
ld.add_action(LaunchDescription([
        DeclareLaunchArgument(
            'ecu_ip',
            default_value='10.30.30.30',
            description='IP address of the ECU'
        ),
        DeclareLaunchArgument(
            'ecu_port',
            default_value='6000',
            description='Port number of the ECU'
        ),
        DeclareLaunchArgument(
            'udp_port',
            default_value='9999',
            description='Port number of the UDP server to listen for IMU data from ECU'
        )
    ])
)

log_level_arg = DeclareLaunchArgument(
    'log_level',
    default_value='info',
    description='Logging level (debug, info, warn, error, fatal)'
)

ld.add_action(log_level_arg)

# Launch lidar node from cspc_lidar package
lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('cspc_lidar'), 'launch', 'lidar.launch.py')
            ),
            launch_arguments={
                'log_level': log_level
            }.items()
        )       
# Launch ros2_control system for driving real motors
motors_control =  IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package_name), 'launch', 'launch_hw.launch.py')
    ),
    launch_arguments={
        'use_sim_time': use_sim_time,
        'ecu_ip': ecu_ip,
        'ecu_port': ecu_port,
        'udp_port': udp_port,
        'log_level': log_level,
    }.items()
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
    arguments=['--ros-args', '--log-level', log_level],
    remappings=[('/image_raw','/camera/image_raw')]

)

# Add all components into the LaunchDescription in the desired sequence.

ld.add_action(motors_control)         # Start all nodes for motors control.
ld.add_action(lidar)                  # Run lidar node
ld.add_action(camera)                 # Start publishing images from camera.
def generate_launch_description():
    return ld
