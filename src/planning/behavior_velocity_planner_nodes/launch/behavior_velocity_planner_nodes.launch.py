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

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with a single component."""
    container = ComposableNodeContainer(
        name='behavior_velocity_planner_nodes_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='behavior_velocity_planner_nodes',
                plugin='autoware::behavior_velocity_planner_nodes::BehaviorVelocityPlannerNode',
                name='behavior_velocity_planner_node_exe'),
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
