import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(get_package_share_directory('argus_manager'), 'config', 'params.yaml')
    return LaunchDescription([
        Node(
            package='argus_teleop_joy',
            executable='teleop_joy_node',
            name='teleop_joy_node',
            parameters=[config],
            output='screen'
        )
    ])
