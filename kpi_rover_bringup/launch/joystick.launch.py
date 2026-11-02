from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    joy_params = PathJoinSubstitution([
        FindPackageShare('kpi_rover_bringup'), 'config', 'joystick.yaml'])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[
            joy_params,
            {'publish_stamped_twist': True, 'frame': 'base_link', 'use_sim_time': use_sim_time},
        ],
        remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel')],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        joy_node,
        teleop_node,
    ])
