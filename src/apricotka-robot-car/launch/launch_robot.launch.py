import os
from launch.actions import LogInfo
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    package_name='apricotka-robot-car'

    rsp_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','rsp.launch.py')]
        ),
        launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'my_controllers.yaml'
    )
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controller_params],
    )
    delayed_controller_manager = TimerAction(period=3.0,actions=[controller_manager])
    delayed_diff_drive_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller'],
            )],
        )
    )
    delayed_joint_broad_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
            )],
        )
    )

    # Launch them all!
    ld = LaunchDescription()
    ld.add_action(rsp_cmd)
    ld.add_action(delayed_controller_manager)
    ld.add_action(delayed_diff_drive_controller)
    ld.add_action(delayed_joint_broad_broadcaster)
    ld.add_action(LogInfo(condition=IfCondition(LaunchConfiguration('use_ros2_control')), msg='Using ROS2 control.'))
    return ld