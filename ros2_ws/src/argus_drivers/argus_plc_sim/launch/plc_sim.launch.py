from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='argus_plc_sim',
            executable='plc_sim_node',
            name='plc_sim_node',
            output='screen',
            parameters=[
                {'publish_frequency_hz': 50.0},
                {'step_per_cycle': 0.02},
                {'target_tolerance': 0.01},
                {'initial_pitch': 0.0},
                {'initial_yaw': 0.0},
            ],
        )
    ])
