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
project(localization_common)

#dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(LOCALIZATION_COMMON_LIB_SRC
    src/initialization.cpp
    src/optimized_registration_summary.cpp
)

ament_auto_add_library(
${PROJECT_NAME} SHARED
        ${LOCALIZATION_COMMON_LIB_SRC}
)
autoware_set_compile_options(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # gtest
  set(LOCALIZATION_COMMON_TEST localization_common_gtest)

  ament_add_gtest(${LOCALIZATION_COMMON_TEST}
    test/test_initialization.hpp
#    test/test_initialization.cpp
    )

  autoware_set_compile_options(${LOCALIZATION_COMMON_TEST})
  target_compile_options(${LOCALIZATION_COMMON_TEST} PRIVATE -Wno-double-promotion -Wno-float-conversion -Wno-deprecated-declarations)
  target_link_libraries(${LOCALIZATION_COMMON_TEST} ${PROJECT_NAME})
  ament_target_dependencies(${LOCALIZATION_COMMON_TEST} "time_utils")
endif()

ament_auto_package()
