from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_plc_sim = LaunchConfiguration('use_plc_sim')

    def include_launch(package_name, launch_file):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare(package_name), '/launch/', launch_file
            ])
        )

    return LaunchDescription([
        DeclareLaunchArgument('use_plc_sim', default_value='false'),
        # include_launch('argus_joystick', 'joystick.launch.py'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('argus_plc_bridge'), '/launch/', 'bridge.launch.py'
            ]),
            condition=UnlessCondition(use_plc_sim),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('argus_plc_sim'), '/launch/', 'plc_sim.launch.py'
            ]),
            condition=IfCondition(use_plc_sim),
        ),
        include_launch('argus_manager', 'manager.launch.py'),
    ])
