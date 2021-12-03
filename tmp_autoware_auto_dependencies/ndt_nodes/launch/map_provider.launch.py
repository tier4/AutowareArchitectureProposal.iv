# Copyright 2020 the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.

import os

from ament_index_python import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():

    map_data_prefix = get_package_share_directory('autoware_demos')

    # map_provider parameter file definition
    ndt_map_provider_file_path = os.path.join(
        get_package_share_directory('autoware_auto_launch'),
        "param",
        "map_publisher.param.yaml")
    map_provider_param_file = LaunchConfiguration(
        "params", default=[ndt_map_provider_file_path])

    # map provide map file arguments
    map_yaml_file_path = os.path.join(
        map_data_prefix, "data/autonomoustuff_parking_lot_lgsvl.yaml"
    )
    map_pcd_file_path = os.path.join(
        map_data_prefix, "data/autonomoustuff_parking_lot_lgsvl.pcd"
    )
    map_yaml_file_param = DeclareLaunchArgument(
        'map_yaml_file_arg',
        default_value=map_yaml_file_path,
        description='Map YAML file describing map origin'
    )
    map_pcd_file_param = DeclareLaunchArgument(
        'map_pcd_file_arg',
        default_value=map_pcd_file_path,
        description='Map PCD file containing 3D point cloud data'
    )

    # map_provide node execution definition
    map_provider_node_runner = launch_ros.actions.Node(
        package="ndt_nodes",
        executable="ndt_map_publisher_exe",
        namespace="localization",
        parameters=[map_provider_param_file,
                    {"map_yaml_file": LaunchConfiguration('map_yaml_file_arg')},
                    {"map_pcd_file": LaunchConfiguration('map_pcd_file_arg')}])

    # map downsampler paramter file definition
    ndt_map_downsampler_file_path = os.path.join(
        get_package_share_directory('ndt_nodes'),
        'param',
        'pcl_map_voxel_grid_downsample.param.yaml')
    map_downsampler_param_file = launch.substitutions.LaunchConfiguration(
        'params', default=[ndt_map_downsampler_file_path])

    # map downsample node execution definition
    map_downsampler_node_runner = launch_ros.actions.Node(
        package='voxel_grid_nodes',
        executable='voxel_grid_node_exe',
        parameters=[map_downsampler_param_file],
        remappings=[
            ("points_in", "viz_ndt_map"),
            ("points_downsampled", "viz_ndt_map_downsampled")
        ])

    # require a map file location
    return launch.LaunchDescription([
        map_yaml_file_param,
        map_pcd_file_param,
        map_provider_node_runner,
        map_downsampler_node_runner])
