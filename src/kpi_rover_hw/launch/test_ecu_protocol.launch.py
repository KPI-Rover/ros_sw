from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(DeclareLaunchArgument(
        'serial_device',
        default_value='/dev/ttyAMA2',
        description='Serial device for ECU communication'
    ))
    
    declared_arguments.append(DeclareLaunchArgument(
        'baud_rate',
        default_value='921600',
        description='Baud rate for serial communication'
    ))

    serial_device = LaunchConfiguration('serial_device')
    baud_rate = LaunchConfiguration('baud_rate')

    # Ensure the executable exists (it should be installed by CMakeLists.txt)
    test_node = Node(
        package='kpi_rover_hw',
        executable='test_ecu_protocol',
        name='test_ecu_protocol',
        output='screen',
        parameters=[{
            'serial_device': serial_device,
            'baud_rate': baud_rate
        }]
    )

    return LaunchDescription(declared_arguments + [
        test_node
    ])
