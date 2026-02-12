from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. joystick_node
        Node(
            package='argus_joystick',
            executable='joystick_node',
            name='joystick_node',
            output='screen'
        ),
        # 2. plc_bridge_node
        Node(
            package='argus_plc_bridge',
            executable='plc_bridge_node',
            name='plc_bridge_node',
            parameters=[{'plc_ip': '192.168.1.222', 'frequency': 10, 'remote_port': 57446}],
            output='screen'
        ),
        # 3. manager_node
        Node(
            package='argus_manager',
            executable='manager_node',
            name='argus_manager_node',
            parameters=[{'watchdog_timeout_ms': 500}],
            output='screen'
        )
    ])
