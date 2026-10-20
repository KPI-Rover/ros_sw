"""
Launch file for the KPIRover simulation. Its parts are switched with arguments:
- gazebo: Gazebo server with the selected world.
- robot: the robot (spawn_robot.launch.py) with EKF, SLAM toolbox and navigation.
- ui: Gazebo GUI and RViz (launch_sim_ui.launch.py).

Examples:
    ros2 launch kpi_rover launch_sim.launch.py world:=garden world_args:="grass:=0.5 mud:=0.25" start:=road_a
    ros2 launch kpi_rover launch_sim.launch.py world:=maze robot:=false
    ros2 launch kpi_rover launch_sim.launch.py gazebo:=false x_pose:=1.5 y_pose:=-2 yaw_pose:=1.57

The last one adds the robot to a world that is already running.
'python3 worlds/lib/world_info.py' lists the worlds and their arguments.

All nodes use simulated time.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, logging
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

package_dir = get_package_share_directory('kpi_rover')
worlds_dir = os.path.join(package_dir, 'worlds')
sys.path.append(os.path.join(worlds_dir, 'lib'))
import world_info  # noqa: E402

POSE = ('x_pose', 'y_pose', 'z_pose', 'yaw_pose')

# Height above a start point the robot is spawned at, so it drops onto uneven ground instead of into it.
START_LIFT = 0.03


def include(path, condition=None, **launch_arguments):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments=launch_arguments.items(), condition=condition)


def world_setup(context):
    """Build the selected world, start Gazebo with it and set the spawn pose from the 'start' point."""
    cfg = context.launch_configurations
    if not IfCondition(LaunchConfiguration('gazebo')).evaluate(context) and not cfg['start']:
        return []  # the robot goes into a running world at the given pose

    args = world_info.parse_args(cfg['world_args'])
    args.update({name: cfg[name] for name in ('floor', 'marking') if cfg[name]})
    world = world_info.build(cfg['world'], args, worlds_dir)
    # Tools (e.g. a GUI making a map of the generated world) read the world file from this line.
    logging.get_logger('launch_sim').info(f'World file: {world}')

    actions = [include(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'),
                       condition=IfCondition(LaunchConfiguration('gazebo')),
                       gz_args=f'-r -s -v4 {world}', on_exit_shutdown='true')]
    if cfg['start']:
        x, y, z, yaw = world_info.start_pose(world, cfg['start'])
        actions += [SetLaunchConfiguration(name, str(value)) for name, value in zip(POSE, (x, y, z + START_LIFT, yaw))]
    return actions


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    world = OpaqueFunction(function=world_setup)

    robot = GroupAction(condition=IfCondition(LaunchConfiguration('robot')), actions=[
        include(os.path.join(package_dir, 'launch', 'spawn_robot.launch.py'),
                **{name: LaunchConfiguration(name) for name in POSE}),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(package_dir, 'config', 'ekf.yaml'), {'use_sim_time': use_sim_time}],
        ),
        include(os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'),
                slam_params_file=os.path.join(package_dir, 'config', 'slam_toolbox_mapping.yaml'),
                use_sim_time=use_sim_time),
        include(os.path.join(package_dir, 'launch', 'navigation.launch.py'), use_sim_time=use_sim_time),
    ])

    ui = include(os.path.join(package_dir, 'launch', 'launch_sim_ui.launch.py'),
                 condition=IfCondition(LaunchConfiguration('ui')), use_sim_time=use_sim_time)

    arguments = [
        ('use_sim_time', 'true', 'Use simulation (Gazebo) clock if true'),
        ('gazebo', 'true', 'Start Gazebo with the world; false to add the robot to a running one'),
        ('robot', 'true', 'Spawn the robot with its navigation'),
        ('ui', 'true', 'Start the Gazebo GUI and RViz'),
        ('world', 'industrial-warehouse', f"World name ({', '.join(world_info.world_files(worlds_dir))}) or path to an SDF file"),
        ('world_args', '', "Arguments of a .sdf.xacro world as 'name:=value' pairs separated by spaces"),
        ('floor', '', f"Floor of a .sdf.xacro world ({', '.join(world_info.options('floors', worlds_dir))})"),
        ('marking', '', f"Marking of a .sdf.xacro world ({', '.join(world_info.options('markings', worlds_dir))})"),
        ('start', '', 'Start point of the world to spawn the robot at, instead of the pose'),
        ('x_pose', '0.0', 'Robot spawn X position'),
        ('y_pose', '0.0', 'Robot spawn Y position'),
        ('z_pose', '0.0', 'Robot spawn Z position'),
        ('yaw_pose', '0.0', 'Robot spawn yaw in radians'),
    ]

    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=text) for name, default, text in arguments]
        + [world, robot, ui]
    )
