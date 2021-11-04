# Copyright 2021 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    def add_launch_arg(name: str, default_value=None):
        return DeclareLaunchArgument(name, default_value=default_value)

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    composable_nodes = [
        ComposableNode(
            package="pointcloud_to_laserscan",
            plugin="pointcloud_to_laserscan::PointCloudToLaserScanNode",
            name="pointcloud_to_laserscan_node",
            remappings=[
                ("~/input/pointcloud", LaunchConfiguration("input/obstacle_pointcloud")),
                ("~/output/laserscan", LaunchConfiguration("output/laserscan")),
                ("~/output/pointcloud", LaunchConfiguration("output/pointcloud")),
                ("~/output/ray", LaunchConfiguration("output/ray")),
                ("~/output/stixel", LaunchConfiguration("output/stixel")),
            ],
            parameters=[
                {
                    "target_frame": "base_link",  # Leave disabled to output scan in pointcloud frame
                    "transform_tolerance": 0.01,
                    "min_height": 0.0,
                    "max_height": 2.0,
                    "angle_min": -3.141592,  # -M_PI
                    "angle_max": 3.141592,  # M_PI
                    "angle_increment": 0.00436332222,  # 0.25*M_PI/180.0
                    "scan_time": 0.0,
                    "range_min": 1.0,
                    "range_max": 200.0,
                    "use_inf": True,
                    "inf_epsilon": 1.0,
                    # Concurrency level, affects number of pointclouds queued for processing
                    # and number of threads used
                    # 0 : Detect number of cores
                    # 1 : Single threaded
                    # 2->inf : Parallelism level
                    "concurrency_level": 1,
                }
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        ),
        ComposableNode(
            package="laserscan_to_occupancy_grid_map",
            plugin="occupancy_grid_map::OccupancyGridMapNode",
            name="occupancy_grid_map_node",
            remappings=[
                ("~/input/laserscan", LaunchConfiguration("output/laserscan")),
                ("~/input/obstacle_pointcloud", LaunchConfiguration("input/obstacle_pointcloud")),
                ("~/input/raw_pointcloud", LaunchConfiguration("input/raw_pointcloud")),
                ("~/output/occupancy_grid_map", LaunchConfiguration("output")),
            ],
            parameters=[
                {
                    "map_resolution": 0.5,
                    "use_height_filter": True,
                    "input_obstacle_pointcloud": LaunchConfiguration("input_obstacle_pointcloud"),
                    "input_obstacle_and_raw_pointcloud": LaunchConfiguration(
                        "input_obstacle_and_raw_pointcloud"
                    ),
                }
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        ),
    ]

    occupancy_grid_map_container = ComposableNodeContainer(
        condition=LaunchConfigurationEquals("container", ""),
        name="occupancy_grid_map_container",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=composable_nodes,
        output="screen",
    )

    load_composable_nodes = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals("container", ""),
        composable_node_descriptions=composable_nodes,
        target_container=LaunchConfiguration("container"),
    )

    return LaunchDescription(
        [
            add_launch_arg("container", ""),
            add_launch_arg("use_multithread", "false"),
            add_launch_arg("use_intra_process", "false"),
            add_launch_arg("input/obstacle_pointcloud", "no_ground/oneshot/pointcloud"),
            add_launch_arg("input/raw_pointcloud", "concatenated/pointcloud"),
            add_launch_arg("output", "occupancy_grid"),
            add_launch_arg("output/laserscan", "virtual_scan/laserscan"),
            add_launch_arg("output/pointcloud", "virtual_scan/pointcloud"),
            add_launch_arg("output/ray", "virtual_scan/ray"),
            add_launch_arg("output/stixel", "virtual_scan/stixel"),
            add_launch_arg("input_obstacle_pointcloud", "false"),
            add_launch_arg("input_obstacle_and_raw_pointcloud", "true"),
            set_container_executable,
            set_container_mt_executable,
            occupancy_grid_map_container,
            load_composable_nodes,
        ]
    )
