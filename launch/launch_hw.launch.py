import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    sim_mode = LaunchConfiguration('sim_mode', default='false')
    ecu_ip = LaunchConfiguration('ecu_ip')
    ecu_port = LaunchConfiguration('ecu_port')
    udp_port = LaunchConfiguration('udp_port')

    ld = LaunchDescription([
        DeclareLaunchArgument(
            'ecu_ip',
            default_value='10.30.30.30',
            description='IP address of the ECU'
        ),
        DeclareLaunchArgument(
            'ecu_port',
            default_value='6000',
            description='Port number of the ECU'
        ),
        DeclareLaunchArgument(
            'udp_port',
            default_value='9999',
            description='Port number of the UDP server to listen for IMU data from ECU'
        )
    ])

    # Package paths
    package_name = 'kpi_rover'
    pkg_share = get_package_share_directory(package_name)
    urdf_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')

    # Generate robot description (Xacro)
    robot_description = Command([
        'xacro ', urdf_file,
        ' use_sim_time:=', use_sim_time,
        ' sim_mode:=', sim_mode,
        ' ecu_ip:=', ecu_ip,
        ' ecu_port:=', ecu_port,
        ' udp_port:=', udp_port
    ])

    # Controller configuration
    controllers_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'kpi_rover_controllers.yaml'
    ])

    # Start robot state publisher
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    ))

    # Start controller manager with additional parameters
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_config,
            {'ecu_ip': ecu_ip},
            {'ecu_port': ecu_port},
            {'udp_port': udp_port}
        ],
        output='screen'
    )
    ld.add_action(controller_manager)

    # Delay controller spawner to ensure controller_manager is ready
    controller_spawner = TimerAction(
        period=5.0,  # Delay to give time for controller_manager to start
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen'
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller'],
                output='screen'
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["imu_broadcaster"],
            )   
        ]
    )
    ld.add_action(controller_spawner)

    return ld
