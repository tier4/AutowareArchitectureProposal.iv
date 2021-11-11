# Copyright 2021 the Autoware Foundation
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

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml
import launch_ros.actions
import ament_index_python

def generate_launch_description():
    """Generate launch description with a single component."""
    # container = ComposableNodeContainer(
    #     name='behavior_velocity_planner_nodes_container',
    #     namespace='',
    #     package='rclcpp_components',
    #     executable='component_container',
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package='behavior_velocity_planner_nodes',
    #             plugin='autoware::planning::behavior_velocity_planner_nodes::BehaviorVelocityPlannerNode',
    #             name='behavior_velocity_planner_node_exe'),
    #     ],
    #     output='screen',
    # )

    # behavior velocity planner
    vehicle_constants_param_path = os.path.join(
        get_package_share_directory('behavior_velocity_planner_nodes'),
        'config',
        'vehicle_constants.param.yaml',
    )
    with open(vehicle_constants_param_path, 'r') as f:
        vehicle_constants_param = yaml.safe_load(f)['/**']['ros__parameters']

    blind_spot_param_path = os.path.join(
        get_package_share_directory('behavior_velocity_planner_nodes'),
        'config',
        'blind_spot.param.yaml',
    )
    with open(blind_spot_param_path, 'r') as f:
        blind_spot_param = yaml.safe_load(f)['/**']['ros__parameters']

    crosswalk_param_path = os.path.join(
        get_package_share_directory('behavior_velocity_planner_nodes'),
        'config',
        'crosswalk.param.yaml',
    )
    with open(crosswalk_param_path, 'r') as f:
        crosswalk_param = yaml.safe_load(f)['/**']['ros__parameters']

    detection_area_param_path = os.path.join(
        get_package_share_directory('behavior_velocity_planner_nodes'),
        'config',
        'detection_area.param.yaml',
    )
    with open(detection_area_param_path, 'r') as f:
        detection_area_param = yaml.safe_load(f)['/**']['ros__parameters']

    intersection_param_path = os.path.join(
        get_package_share_directory('behavior_velocity_planner_nodes'),
        'config',
        'intersection.param.yaml',
    )
    with open(intersection_param_path, 'r') as f:
        intersection_param = yaml.safe_load(f)['/**']['ros__parameters']

    stop_line_param_path = os.path.join(
        get_package_share_directory('behavior_velocity_planner_nodes'),
        'config',
        'stop_line.param.yaml',
    )
    with open(stop_line_param_path, 'r') as f:
        stop_line_param = yaml.safe_load(f)['/**']['ros__parameters']

    # traffic_light_param_path = os.path.join(
    #     get_package_share_directory('behavior_velocity_planner_nodes'),
    #     'config',
    #     'traffic_light.param.yaml',
    # )
    # with open(traffic_light_param_path, 'r') as f:
    #     traffic_light_param = yaml.safe_load(f)['/**']['ros__parameters']

    # virtual_traffic_light_param_path = os.path.join(
    #     get_package_share_directory('behavior_velocity_planner_nodes'),
    #     'config',
    #     'virtual_traffic_light.param.yaml',
    # )
    # with open(virtual_traffic_light_param_path, 'r') as f:
    #     virtual_traffic_light_param = yaml.safe_load(f)['/**']['ros__parameters']

    behavior_velocity_planner_nodes = launch_ros.actions.Node(
        package='behavior_velocity_planner_nodes',
        executable='behavior_velocity_planner_nodes_exe',
        name='behavior_velocity_planner_nodes',
        namespace='',
        output='screen',
        parameters=[
            {
                'launch_stop_line': True,
                'launch_crosswalk': True,
                'launch_traffic_light': True,
                'launch_intersection': True,
                'launch_blind_spot': True,
                'launch_detection_area': True,
                'forward_path_length': 1000.0,
                'backward_path_length': 5.0,
                'max_accel': -2.8,
                'delay_response_time': 1.3
            },
            vehicle_constants_param,
            blind_spot_param,
            crosswalk_param,
            detection_area_param,
            intersection_param,
            stop_line_param,
        ],
    )

    return launch.LaunchDescription([behavior_velocity_planner_nodes])