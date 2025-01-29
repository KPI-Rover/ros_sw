from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    joy_params = os.path.join(get_package_share_directory('kpi_rover'), 'config', 'joystick.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params],
        )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'publish_stamped_twist': True, 'frame': 'base_link'}],
            remappings=[('/cmd_vel', '/diff_drive_base_controller/cmd_vel')]
        )

    ld = LaunchDescription()

    ld.add_action(joy_node)
    ld.add_action(teleop_node)

    return ld
