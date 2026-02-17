import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(get_package_share_directory('argus_joystick_driver'), 'config', 'params.yaml')
    return LaunchDescription([
        Node(
            package='argus_joystick_driver',
            executable='joystick_node',
            name='argus_joystick_driver',
            parameters=[config],
            output='screen'
        )
    ])
