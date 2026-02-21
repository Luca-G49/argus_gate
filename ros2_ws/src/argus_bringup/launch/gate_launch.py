# Argus Gate - gate_launch.py
# Copyright (c) 2026, Luca G49
# All rights reserved. Licensed under MIT License.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Helper per trovare e includere i launch file degli altri package
    def include_launch(package_name, launch_file):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare(package_name), '/launch/', launch_file
            ])
        )

    return LaunchDescription([
        #include_launch('argus_joystick', 'joystick.launch.py'),
        include_launch('argus_plc_bridge', 'bridge.launch.py'),
        include_launch('argus_manager', 'manager.launch.py'),
    ])
