from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_bringup = FindPackageShare('kpi_rover_bringup')
    pkg_description = FindPackageShare('kpi_rover_description')

    wheel_radius = LaunchConfiguration('wheel_radius')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')

    declared_arguments = [
        DeclareLaunchArgument(
            'wheel_radius', default_value='0.034',
            description='Wheel radius in meters'),
        DeclareLaunchArgument('x_pose', default_value='0.0', description='Spawn X position'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='Spawn Y position'),
        DeclareLaunchArgument('z_pose', default_value='0.1', description='Spawn Z position'),
        DeclareLaunchArgument('yaw_pose', default_value='0.0', description='Spawn yaw, rad'),
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='Start slam_toolbox'),
        DeclareLaunchArgument(
            'nav', default_value='true',
            description='Start Nav2 (handles the RViz 2D Goal Pose)'),
    ]

    # robot_state_publisher with the sim_mode URDF (gz_ros2_control plugin)
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_description, 'launch', 'description.launch.py'])),
        launch_arguments={
            'use_sim_time': 'true',
            'sim_mode': 'true',
            'wheel_radius': wheel_radius,
        }.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'kpi_rover',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_pose,
        ],
        output='screen',
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([pkg_bringup, 'config', 'gz_bridge.yaml']),
        }],
        output='screen',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--switch-timeout', '60'],
        output='screen',
    )

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--switch-timeout', '60',
            '--controller-ros-args', '-r /diff_drive_controller/odom:=/odom',
        ],
        output='screen',
    )

    # The controller manager only runs once the robot model is in the world
    spawn_controllers = RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot,
        on_exit=[joint_state_broadcaster, diff_drive_controller],
    ))

    # Turns unstamped /cmd_vel (Nav2, teleop_twist_keyboard) into the stamped input of the controller
    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': True, 'frame_id': 'base_link'}],
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', '/diff_drive_controller/cmd_vel'),
        ],
        output='screen',
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('kpi_rover_localization'), 'launch', 'localization.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('kpi_rover_slam'), 'launch', 'slam.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('slam')),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': PathJoinSubstitution([pkg_bringup, 'config', 'navigation.yaml']),
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav')),
    )

    return LaunchDescription(declared_arguments + [
        description,
        spawn_robot,
        gz_bridge,
        spawn_controllers,
        twist_stamper,
        localization,
        slam,
        navigation,
    ])
