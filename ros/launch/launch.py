import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    disable_realsense = LaunchConfiguration('disable_realsense')

    return LaunchDescription([
        DeclareLaunchArgument(
            'disable_realsense',
            default_value='false',
            description='Whether to disable the Realsense camera'
        ),
        # Rosbridge
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('rosbridge_server'),
                    'launch/rosbridge_websocket_launch.xml'
                )
            )
        ),
        # RCS Core
        Node(
            package='rcs_core',
            executable='core'
        ),
        # Auto Nav
        Node(
            package='auto_nav',
            executable='auto_nav'
        ),
        # Pathfinder
        Node(
            package='pathfinder',
            executable='pathfinder',
        ),
        # Realsense
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('realsense2_camera'),
                    'launch/rs_launch.py'
                ),
            ),
            condition=UnlessCondition(disable_realsense)
        ),
    ])
