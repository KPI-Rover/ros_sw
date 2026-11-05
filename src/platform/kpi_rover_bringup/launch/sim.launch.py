from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from kpi_rover_worlds import world_info


def start_pose(context):
    cfg = context.launch_configurations
    if not cfg['start']:
        return []
    world = world_info.build(cfg['world'], {})
    x, y, z, yaw = world_info.start_pose(world, cfg['start'])
    # 3 cm above the start point, so the robot drops onto uneven ground instead of into it
    pose = zip(('x_pose', 'y_pose', 'z_pose', 'yaw_pose'), (x, y, z + 0.03, yaw))
    return [SetLaunchConfiguration('world', world)] + [SetLaunchConfiguration(k, str(v)) for k, v in pose]


def generate_launch_description():
    pkg_bringup = FindPackageShare('kpi_rover_bringup')

    rviz = LaunchConfiguration('rviz')

    declared_arguments = [
        DeclareLaunchArgument(
            'world', default_value='room',
            description='World of kpi_rover_worlds (python3 -m kpi_rover_worlds.world_info) or path to a world file'),
        DeclareLaunchArgument(
            'start', default_value='',
            description='Start point of the world to spawn the robot at, instead of the pose'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz2'),
        DeclareLaunchArgument(
            'teleop', default_value='true',
            description='Start joystick teleop'),
    ]

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bringup, 'launch', 'world.launch.py'])),
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bringup, 'launch', 'sim_robot.launch.py'])),
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bringup, 'launch', 'joystick.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('teleop')),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bringup, 'launch', 'rviz.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(rviz),
    )

    return LaunchDescription(declared_arguments + [
        OpaqueFunction(function=start_pose),
        world,
        robot,
        teleop,
        rviz_launch,
    ])
