import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        # Rosbridge
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('rosbridge_server'),
                    'launch/rosbridge_websocket_launch.xml'
                )
            )
        ),
        # Realsense
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('realsense2_camera'),
                    'launch/rs_launch.py'
                ),
            )
        ),
        # # Realsense Interop
        Node(
            package='realsense_interop',
            executable='realsense_interop',
            parameters=[
                {'realsense_interop.image.jpeg_quality': 15}
            ]
        ),
        # RCS Core
        Node(
            package='rcs_core',
            executable='core'
        )
    ])
