from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        output='screen',
        parameters= [{
        'image_size': [320, 240],
        'video_device':"/dev/video0",
        'pixel_format': "yuv422_yuy2",
        'output_encoding': "rgb8",
}],
        remappings=[('/image_raw','/camera/image_raw')]

    )

    # transport = Node(
    #     package='image_transport',
    #     executable='republish',
    #     name='image_transport_republisher',
    #     parameters= [{
    #     'in_transport': 'raw',
    #     'out_transport': 'ffmpeg',
    #     'output_encoding': "yuv422_yuy2",
    #     }],
    #     remappings=[('in','/camera/image_raw'),
    #                  ('out', '/camera/image_ffmpeg')]
    # )

    ld.add_action(camera)
    #ld.add_action(transport)

    #ros2 run image_transport republish --ros-args -p in_transport:=ffmpeg -p out_transport:=raw --remap in/ffmpeg:=image_raw/ffmpeg --remap out:=image_raw/uncompressed -p "ffmpeg_image_transport.map.hevc_nvenc:=hevc"

    return ld


