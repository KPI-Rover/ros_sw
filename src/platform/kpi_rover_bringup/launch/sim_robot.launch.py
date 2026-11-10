from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
    robot_ns = LaunchConfiguration('robot_ns')
    camera = LaunchConfiguration('camera')

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
        DeclareLaunchArgument(
            'robot_ns', default_value='',
            description='Name of the robot in a simulation shared by several robots: its model, '
                        'Gazebo topics and controller manager. The controller manager topics move under it '
                        'and need a bridge from what runs the shared simulation'),
        DeclareLaunchArgument(
            'load_controllers', default_value='true',
            description='Load the controllers; false when something else loads them'),
        DeclareLaunchArgument(
            'camera', default_value='true',
            description='Add the camera to the model: /camera/image_raw, /camera/camera_info'),
    ]

    # robot_state_publisher with the sim_mode URDF (gz_ros2_control plugin)
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_description, 'launch', 'description.launch.py'])),
        launch_arguments={
            'use_sim_time': 'true',
            'sim_mode': 'true',
            'wheel_radius': wheel_radius,
            'robot_ns': robot_ns,
            'camera': camera,
        }.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', PythonExpression(["'", robot_ns, "' or 'kpi_rover'"]),
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_pose,
        ],
        output='screen',
    )

    # Relative Gazebo topics of the config get the namespace (<robot_ns>/scan), the ROS topics don't
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=robot_ns,
        parameters=[{
            'config_file': PathJoinSubstitution([pkg_bringup, 'config', 'gz_bridge.yaml']),
            'expand_gz_topic_names': True,
        }],
        output='screen',
    )

    # The image goes through ros_gz_image, not gz_bridge.yaml: it adds the image_transport topics (compressed).
    # The Gazebo topic is <robot_ns>/camera/image_raw, the ROS topic /camera/image_raw
    camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[[robot_ns, '/camera/image_raw']],
        remappings=[([robot_ns, '/camera/image_raw'], '/camera/image_raw')],
        output='screen',
        condition=IfCondition(camera),
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', [robot_ns, '/controller_manager'], '--switch-timeout', '60'],
        output='screen',
    )

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '-c', [robot_ns, '/controller_manager'],
            '--switch-timeout', '60',
            '--controller-ros-args', '-r /diff_drive_controller/odom:=/odom',
        ],
        output='screen',
    )

    # The controller manager only runs once the robot model is in the world
    spawn_controllers = RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot,
        on_exit=[joint_state_broadcaster, diff_drive_controller],
    ), condition=IfCondition(LaunchConfiguration('load_controllers')))

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
        camera_bridge,
        spawn_controllers,
        twist_stamper,
        localization,
        slam,
        navigation,
    ])
