"""
Launch file for the KPIRover simulation.

This file launches the following components:
- Gazebo simulation server with a specific world file.
- Robot State Publisher (rsp) with ROS2 control enabled.
- Spawner to insert the robot into the simulation at a given pose.
- Controllers for joint state broadcasting and differential drive.
- EKF node for sensor fusion and localization.
- SLAM toolbox for online asynchronous mapping.
- Navigation stack for path planning.
- Gazebo UI and joystick interface for simulation control.

All nodes use simulated time.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Define the common simulation clock parameter
use_sim_time = LaunchConfiguration('use_sim_time', default='true')

package_name = 'kpi_rover'
ros_gz_sim = get_package_share_directory('ros_gz_sim')

ld = LaunchDescription()
ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'))

# Launch Gazebo simulation server with the specified world file.
# This starts the simulation environment.
world = os.path.join(
    get_package_share_directory(package_name),
    'worlds',
    'forest.world'
)
gzserver_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
    ),
    launch_arguments={
        'gz_args': ['-r -s -v4 ', world],
        'on_exit_shutdown': 'true'
    }.items()
)

# Launch the Robot State Publisher (rsp) with ROS2 control enabled.
# This node publishes the state of the robot to TF and other systems.
rsp_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
    ),
    launch_arguments={
        'use_sim_time': use_sim_time,
        'use_ros2_control': 'true'
    }.items()
)

# Spawn the robot into the simulation at a specified position.
# The x, y, and z positions define where the robot appears in the world.
spawn_robot_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package_name), 'launch', 'spawn_robot.launch.py')
    ),
    launch_arguments={
        'x_pose': LaunchConfiguration('x_pose', default='-8.0'),
        'y_pose': LaunchConfiguration('y_pose', default='1.5'),
        'z_pose': LaunchConfiguration('z_pose', default='0.18')
    }.items()
)

# Load the Joint State Broadcaster controller.
# This controller publishes the joint states necessary for robot control.
load_joint_state_broadcaster = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
    output='screen'
)

# Load the Differential Drive controller.
# This controller converts velocity commands into motor commands for robot maneuvering.
load_diff_drive_controller = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_controller'],
    output='screen'
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

# Launch the navigation stack.
# Provides path planning and obstacle avoidance for autonomous robot movement.
nav = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package_name), 'launch', 'navigation.launch.py')
    ),
    launch_arguments={'use_sim_time': use_sim_time}.items()
)

# Include the Gazebo UI .
# This provides a graphical interface for controlling the simulation environment.
launch_sim_ui = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package_name), 'launch', 'launch_sim_ui.launch.py')
    ),
    launch_arguments={'use_sim_time': use_sim_time}.items()
)

joystick_launch_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package_name), 'launch', 'joystick.launch.py')
    ),
    launch_arguments={'use_sim_time': use_sim_time}.items()
)

# Add all components into the LaunchDescription in the desired sequence.
ld.add_action(gzserver_cmd)           # Start Gazebo simulation server.
ld.add_action(rsp_cmd)                # Start the robot state publisher.
ld.add_action(spawn_robot_cmd)        # Spawn the robot into the simulation.
ld.add_action(load_joint_state_broadcaster)  # Load joint state controller.
ld.add_action(load_diff_drive_controller)    # Load differential drive controller.
ld.add_action(ekf)                    # Run EKF for sensor fusion and localization.
ld.add_action(slam_toolbox_map)       # Run SLAM toolkit for mapping.
ld.add_action(nav)                    # Start navigation stack.
ld.add_action(launch_sim_ui)          # Launch Gazebo UI
ld.add_action(joystick_launch_cmd)    # Launch joystick interface

def generate_launch_description():
    return ld
