# Argus Gate - argus_plc_bridge/launch/bridge.launch.py
# Copyright (c) 2026, Luca-G49
# All rights reserved. Licensed under MIT License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(get_package_share_directory('argus_plc_bridge'), 'config', 'params.yaml')
    return LaunchDescription([
        Node(
            package='argus_plc_bridge',
            executable='plc_bridge_node',
            name='argus_plc_bridge',
            parameters=[config],
            output='screen'
        )
    ])
