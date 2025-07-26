import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

package_name = 'kpi_rover'

ld = LaunchDescription()

camera = Node(
    package='v4l2_camera',
    executable='v4l2_camera_node',
    name='camera_node',
    output='screen',
    parameters= [{
        'image_size': [320, 320],
        'video_device':"/dev/video0",
        'output_encoding': "yuv422_yuy2"}],
    remappings=[('/image_raw','/camera/image_raw')]
)

yolo = Node(
    package=package_name,
    executable='object_detector.py',
    name='object_detector_node',
    output='screen',
)

transport_compressed = Node(
        package='image_transport',
        executable='republish',
        name='image_republisher',
        output='screen',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/detections/image'),
            ('out/compressed', '/detections/image/compressed')
        ]
)

ld.add_action(camera)
ld.add_action(yolo)
ld.add_action(transport_compressed)

def generate_launch_description():
    return ld