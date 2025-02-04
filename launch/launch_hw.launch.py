import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    pkg_path = get_package_share_directory('kpi_rover')

    # Include the robot state publisher launch file
    rsp_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kpi_rover'),
                'launch',
                'rsp.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': use_ros2_control}.items()
    )

    # Node to launch RViz
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_path, 'description', 'robot.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Include the joystick launch file
    joystick_launch_file = PathJoinSubstitution([
        FindPackageShare('kpi_rover'),
        'launch',
        'joystick.launch.py'
    ])
    node_joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joystick_launch_file),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('kpi_rover'),
            'config',
            'kpi_rover_controllers.yaml',
        ]
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
        output='both',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_base_controller'],
    )


    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'
        )
    ]

    # Define the nodes to be launched
    nodes = [
        rsp_cmd,
        node_rviz,
        node_joystick,
        control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner
    ]

    # Return the LaunchDescription with declared arguments and nodes
    return LaunchDescription(declared_arguments + nodes)
