import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, GroupAction
from launch_ros.actions import SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    package = get_package_share_directory('ros_sw')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    
    world = os.path.join(
        package,
        'worlds',
        'empty.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )
    rsp_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            package,'launch','rsp.launch.py')]
        ),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )
    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package,'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_controller'],
        output='screen'
    )

    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(os.path.join(package, 'launch'), 'dual_ekf_navsat.launch.py'))
    )

    nav2_bringup = get_package_share_directory('nav2_bringup')
    configured_params = RewrittenYaml(
        source_file=os.path.join(os.path.join(package, "config"), "nav2_no_map_params.yaml"), 
        root_key="", param_rewrites="", convert_types=True
    )
    navigation2_cmd = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/diff_drive_controller/cmd_vel'),
            SetRemap(src='/odom', dst='/diff_drive_controller/odom'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "True",
                    "params_file": configured_params,
                    "autostart": "True",
                }.items()
            )
        ]
    )

    use_rviz = LaunchConfiguration('use_rviz')
    use_mapviz = LaunchConfiguration('use_mapviz')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start RVIZ')

    declare_use_mapviz_cmd = DeclareLaunchArgument(
        'use_mapviz',
        default_value='False',
        description='Whether to start mapviz')

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, "launch", 'rviz_launch.py')),
        condition=IfCondition(use_rviz)
    )

    mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(os.path.join(package, 'launch'), 'mapviz.launch.py')),
        condition=IfCondition(use_mapviz)
    )

    # Launch them all!
    ld = LaunchDescription()

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(rsp_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(load_joint_state_broadcaster)
    ld.add_action(load_diff_drive_controller)

    ld.add_action(robot_localization_cmd)
    ld.add_action(navigation2_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(declare_use_mapviz_cmd)
    ld.add_action(mapviz_cmd)

    return ld