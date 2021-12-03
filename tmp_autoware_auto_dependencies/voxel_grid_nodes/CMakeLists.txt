# Copyright 2017-2018 the Autoware Foundation
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

cmake_minimum_required(VERSION 3.5)

### Export headers
project(voxel_grid_nodes)

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### build node as library
set(VOXEL_GRID_NODE_LIB voxel_grid_node)
ament_auto_add_library(${VOXEL_GRID_NODE_LIB} SHARED
  include/voxel_grid_nodes/algorithm/voxel_cloud_base.hpp
  include/voxel_grid_nodes/algorithm/voxel_cloud_approximate.hpp
  include/voxel_grid_nodes/algorithm/voxel_cloud_centroid.hpp
  include/voxel_grid_nodes/visibility_control.hpp
  src/algorithm/voxel_cloud_base.cpp
  src/algorithm/voxel_cloud_approximate.cpp
  src/algorithm/voxel_cloud_centroid.cpp
  include/voxel_grid_nodes/voxel_cloud_node.hpp
  src/voxel_cloud_node.cpp
)
autoware_set_compile_options(${VOXEL_GRID_NODE_LIB})

set(NODE_NAME ${VOXEL_GRID_NODE_LIB}_exe)
rclcpp_components_register_node(${VOXEL_GRID_NODE_LIB}
  PLUGIN "autoware::perception::filters::voxel_grid_nodes::VoxelCloudNode"
  EXECUTABLE ${NODE_NAME}
)

if(BUILD_TESTING)
  # run linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # gtest
  set(VOXEL_GRID_NODE_GTEST voxel_algorithm_gtest)
  ament_add_gtest(${VOXEL_GRID_NODE_GTEST} test/test_voxel_grid_nodes.cpp)
  autoware_set_compile_options(${VOXEL_GRID_NODE_GTEST})
  target_link_libraries(${VOXEL_GRID_NODE_GTEST} ${VOXEL_GRID_NODE_LIB})
  target_include_directories(${VOXEL_GRID_NODE_GTEST} PRIVATE "test/include" "include")

  add_ros_test(
    test/voxel_grid_fuzzing.test.py
    TIMEOUT "120"
  )

  find_package(autoware_testing REQUIRED)
  add_smoke_test(${PROJECT_NAME} ${NODE_NAME})
endif()

# Ament Exporting
ament_auto_package(INSTALL_TO_SHARE
  param)
