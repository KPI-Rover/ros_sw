"""
Launch file putting the KPIRover into a running Gazebo simulation:
- Robot State Publisher (rsp) with ROS2 control enabled.
- The robot model spawned at a pose (x_pose, y_pose, z_pose, yaw_pose).
- Bridges of the robot topics and the camera image.
- Controllers for joint state broadcasting and differential drive.

In a simulation shared by several robots each one has its own robot_ns:
the model name, the Gazebo topics of its sensors and the namespace of its controller manager.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    package_name='kpi_rover'

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.0')
    yaw_pose = LaunchConfiguration('yaw_pose', default='0.0')
    robot_ns = LaunchConfiguration('robot_ns')
    load_controllers = LaunchConfiguration('load_controllers')

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Robot spawn X position')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Robot spawn Y position')
    
    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose', default_value='0.0',
        description='Robot spawn Z position')

    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw_pose', default_value='0.0',
        description='Robot spawn yaw in radians')

    declare_robot_ns_cmd = DeclareLaunchArgument(
        'robot_ns', default_value='',
        description='Namespace of the robot in a simulation shared by several robots; empty for one robot')

    declare_load_controllers_cmd = DeclareLaunchArgument(
        'load_controllers', default_value='true',
        description='Load the controllers; false when something else loads them')

    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', PythonExpression(["'", robot_ns, "' or 'my_bot'"]),
            '-topic', '/robot_description',
            '-allow_renaming', 'true',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_pose
        ],
        output='screen',
    )

    bridge_params = os.path.join(
        get_package_share_directory(package_name),
        'params',
        'robot_bridge.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=robot_ns,
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
            '-p',
            'expand_gz_topic_names:=true',
        ],
        output='screen',
    )

    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        # <robot_ns>/camera/image_raw in Gazebo, /camera/image_raw in ROS
        arguments=[[robot_ns, '/camera/image_raw']],
        remappings=[([robot_ns, '/camera/image_raw'], '/camera/image_raw')],
        output='screen',
    )

    # Launch the Robot State Publisher (rsp) with ROS2 control enabled.
    # This node publishes the state of the robot to TF and other systems.
    rsp_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': 'true',
            'robot_ns': robot_ns,
        }.items()
    )

    # Load the Joint State Broadcaster controller.
    # This controller publishes the joint states necessary for robot control.
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen',
        condition=IfCondition(load_controllers)
    )

    # Load the Differential Drive controller.
    # This controller converts velocity commands into motor commands for robot maneuvering.
    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_controller'],
        output='screen',
        condition=IfCondition(load_controllers)
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)
    ld.add_action(declare_yaw_cmd)
    ld.add_action(declare_robot_ns_cmd)
    ld.add_action(declare_load_controllers_cmd)
    # Add any conditioned actions
    ld.add_action(rsp_cmd)
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_gazebo_ros_image_bridge_cmd)
    ld.add_action(load_joint_state_broadcaster)
    ld.add_action(load_diff_drive_controller)
    return ld