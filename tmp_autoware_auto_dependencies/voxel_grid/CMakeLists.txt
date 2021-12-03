# Copyright 2017-2019 the Autoware Foundation
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
project(voxel_grid)

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/voxel_grid/config.hpp
  include/voxel_grid/voxel.hpp
  include/voxel_grid/voxels.hpp
  include/voxel_grid/voxel_grid.hpp
  include/voxel_grid/visibility_control.hpp
  src/config.cpp
  src/voxels.cpp
  src/voxel_grid.cpp
)
autoware_set_compile_options(${PROJECT_NAME})

if(BUILD_TESTING)
  # run linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # gtest
  set(VOXEL_GRID_GTEST voxel_grid_gtest)

  ament_add_gtest(${VOXEL_GRID_GTEST} test/src/test_voxel_grid.cpp)
  autoware_set_compile_options(${VOXEL_GRID_GTEST})
  target_compile_options(${VOXEL_GRID_GTEST} PRIVATE -Wno-sign-conversion -Wno-double-promotion)
  target_link_libraries(${VOXEL_GRID_GTEST} ${PROJECT_NAME})
  target_include_directories(${VOXEL_GRID_GTEST} PRIVATE "test/include" "include")
  ament_target_dependencies(${VOXEL_GRID_GTEST} "geometry_msgs")
endif()

# Ament Exporting
ament_auto_package()
