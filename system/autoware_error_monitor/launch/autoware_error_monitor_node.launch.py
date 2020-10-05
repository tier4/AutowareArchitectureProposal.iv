from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackage
from pathlib import Path

import os

context = LaunchContext()

def find_pack(package_name):
    """Return the absolute path to the share directory of the given package."""
    return os.path.join(Path(FindPackage(package_name).perform(context)), 'share', package_name)

def generate_launch_description():

    autoware_error_monitor_param_file = os.path.join(find_pack('autoware_error_monitor'), 'config/autoware_error_monitor.param.yaml')

    # Declare launch parameter for config_file
    autoware_error_monitor_param = DeclareLaunchArgument(
        'autoware_error_monitor_param_file',
        default_value=autoware_error_monitor_param_file,
        description='Path to the config file used in the autoware_error_monitor node'
    )

    autoware_error_monitor = Node(
        package='autoware_error_monitor',
        node_executable='autoware_error_monitor_exe',
        node_name='autoware_error_monitor',
        node_namespace='autoware_error_monitor',
        output='screen',
        parameters=[LaunchConfiguration('autoware_error_monitor_param_file')],
        remappings=[
            ('input/diag_array', '/diagnostics_agg'),
            ('output/driving_capability', 'output/driving_capability'),
        ]
    )

    return LaunchDescription([
        autoware_error_monitor_param,
        autoware_error_monitor
    ])
    