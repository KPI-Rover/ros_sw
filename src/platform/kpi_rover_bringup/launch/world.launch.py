from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_description = FindPackageShare('kpi_rover_description')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')

    declared_arguments = [
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([FindPackageShare('kpi_rover_worlds'), 'worlds', 'room', 'room.world']),
            description='Path to the Gazebo world file'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start the Gazebo GUI (otherwise headless server only)'),
    ]

    # Lets Gazebo resolve model://kpi_rover_description/meshes/...
    gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        PathJoinSubstitution([pkg_description, '..']),
    )

    # '-s' runs the server only (headless)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': [
                '-r -v3 ',
                PythonExpression(["'' if '", gui, "' == 'true' else '-s '"]),
                world,
            ],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    return LaunchDescription(declared_arguments + [
        gz_resource_path,
        gazebo,
    ])
