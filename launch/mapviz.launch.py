import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="mapviz",
            executable="mapviz",
            name="mapviz",
            parameters=[{"config": os.path.join(get_package_share_directory('ros_sw'),
                                                 "config", "gps_wpf_demo.mvc")}]
        ),
        launch_ros.actions.Node(
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
            remappings=[
                ("fix", "gps/fix"),
            ],
        ),
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="swri_transform",
            arguments=["--x", "0", "--y", "0", "--z", "0", 
               "--roll", "0", "--pitch", "0", "--yaw", "0",
               "--frame-id", "map", "--child-frame-id", "origin"]
        )
    ])
