from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='argus_flet_gui',
            executable='argus_flet_gui',
            name='argus_flet_gui',
            output='screen',
        )
    ])
