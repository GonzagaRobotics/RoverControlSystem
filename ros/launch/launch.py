import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_realsense = LaunchConfiguration('use_realsense')
    use_auto_nav = LaunchConfiguration('use_auto_nav')
    use_rtc = LaunchConfiguration('use_rtc')

    return LaunchDescription([
        # Realsense
        DeclareLaunchArgument(
            'use_realsense',
            default_value='false',
            description='Whether to start nodes for the Realsense'
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('realsense2_camera'),
                    'launch/rs_launch.py'
                ),
            ),
            condition=IfCondition(use_realsense)
        ),
        # WebRTC
        DeclareLaunchArgument(
            'use_rtc',
            default_value='false',
            description='Whether to start nodes for WebRTC'
        ),
        Node(
            package='web_rtc',
            executable='web_rtc',
            condition=IfCondition(use_rtc)
        ),
        # Auto Nav
        DeclareLaunchArgument(
            'use_auto_nav',
            default_value='false',
            description='Whether to start nodes for AutoNav'
        ),
        Node(
            package='auto_nav',
            executable='auto_nav',
            condition=IfCondition(use_auto_nav)
        ),
        Node(
            package='pathfinder',
            executable='pathfinder',
            condition=IfCondition(use_auto_nav)
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
    ])
