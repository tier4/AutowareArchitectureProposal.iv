# Copyright 2019 The Autoware Foundation
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
project(localization_nodes)

#dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# includes
set(LOCALIZATION_NODES_LIB_SRC
    src/localization_node.cpp
)

ament_auto_add_library(
${PROJECT_NAME} SHARED
        ${LOCALIZATION_NODES_LIB_SRC}
)
autoware_set_compile_options(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # gtest
  set(LOCALIZATION_NODE_TEST localization_node_gtest)

  ament_add_gtest(${LOCALIZATION_NODE_TEST}
          test/test_relative_localizer_node.hpp
          test/test_relative_localizer_node.cpp)
  autoware_set_compile_options(${LOCALIZATION_NODE_TEST})
  target_compile_options(${LOCALIZATION_NODE_TEST} PRIVATE -Wno-sign-conversion -Wno-conversion)
  target_link_libraries(${LOCALIZATION_NODE_TEST} ${PROJECT_NAME})
endif()

ament_auto_package()
