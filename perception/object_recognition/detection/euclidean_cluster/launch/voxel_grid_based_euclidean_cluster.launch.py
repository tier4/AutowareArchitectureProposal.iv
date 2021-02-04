# Copyright 2020 Tier IV, Inc. All rights reserved.
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
from launch import LaunchContext
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression


context = LaunchContext()


def generate_launch_description():

  ns = 'euclidean_cluster'
  pkg = 'euclidean_cluster'

  # declare launch arguments
  input_pointcloud_param = DeclareLaunchArgument(
      'input_pointcloud',
      default_value='/sensing/lidar/no_ground/pointcloud')

  input_map_param = DeclareLaunchArgument(
      'input_map',
      default_value='/map/pointcloud_map')

  output_clusters_param = DeclareLaunchArgument(
      'output_clusters',
      default_value='clusters')

  use_pointcloud_map_param = DeclareLaunchArgument(
      'use_pointcloud_map',
      default_value='false')

  # set voxel grid filter as a component
  voxel_grid_filter_component = ComposableNode(
      package='voxel_grid_filter',
      plugin='pcl::VoxelGrid',
      name='voxel_grid_filter',
      remappings=[('input', LaunchConfiguration('input_pointcloud')),
                  ('output', 'voxel_grid_filtered/pointcloud')],
      parameters=[
          {
                'filter_field_name': 'z',
                'filter_limit_min': 0.1,
                'filter_limit_max': 2.5,
                'filter_limit_negative': False,
                'leaf_size': 0.1,
                'input_frame': 'base_link',
                'output_frame': 'base_link',
          }
      ]
  )

  # set compare map filter as a component
  compare_map_filter_component = ComposableNode(
      package='pointcloud_preprocessor',
      plugin='pointcloud_preprocessor::VoxelBasedCompareMapFilterComponent',
      name='voxel_grid_filter',
      remappings=[('input', 'voxel_grid_filtered/pointcloud'),
                  ('map', LaunchConfiguration('input_map')),
                  ('output', 'compare_map_filtered/pointcloud')]
  )

  # set euclidean cluster as a component
  if IfCondition(LaunchConfiguration('use_pointcloud_map')).evaluate():
      ec_input_topic = 'compare_map_filtered/pointcloud'
  else:
      ec_input_topic = 'voxel_grid_filtered/pointcloud'
  euclidean_cluster_component = ComposableNode(
      package=pkg,
      plugin='euclidean_cluster::VoxelGridBasedEuclideanClusterNodelet',
      name='euclidean_cluster',
      remappings=[('input', ec_input_topic),
                  ('output', LaunchConfiguration('output_clusters'))],
      parameters=[
          {
                'target_frame': 'base_link',
                'tolerance': 0.7,
                'voxel_leaf_size': 0.35,
                'min_points_number_per_voxel': 2,
                'min_cluster_size': 3,
                'max_cluster_size': 3000
          }
      ]
  )

  # set container to run all required components in the same process
  if IfCondition(LaunchConfiguration('use_pointcloud_map')):
    node_list = [
              voxel_grid_filter_component,
              compare_map_filter_component,
              euclidean_cluster_component]
  else:
    node_list = [
            voxel_grid_filter_component,
            euclidean_cluster_component]

  container = ComposableNodeContainer(
      name='euclidean_cluster_container',
      namespace=ns,
      package='rclcpp_components',
      executable='component_container',
      composable_node_descriptions=node_list,
      output='screen',
  )

  return launch.LaunchDescription([
      input_pointcloud_param,
      input_map_param,
      output_clusters_param,
      use_pointcloud_map_param,
      container
  ])