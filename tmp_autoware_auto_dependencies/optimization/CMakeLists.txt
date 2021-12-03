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
project(optimization)

#dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

# includes
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})


set(OPTIMIZATION_LIB_SRC
        src/newtons_method_optimizer.cpp
        src/optimizer_options.cpp
        src/utils.cpp)

set(OPTIMIZATION_LIB_HEADERS
        include/optimization/visibility_control.hpp
        include/optimization/utils.hpp
        include/optimization/optimization_problem.hpp
        include/optimization/optimizer_options.hpp
        include/optimization/optimizer.hpp
        include/optimization/newtons_method_optimizer.hpp
        include/optimization/line_search/line_search.hpp
        include/optimization/line_search/fixed_line_search.hpp
        include/optimization/line_search/more_thuente_line_search.hpp)

ament_auto_add_library(
${PROJECT_NAME} SHARED
        ${OPTIMIZATION_LIB_HEADERS}
        ${OPTIMIZATION_LIB_SRC}
)
autoware_set_compile_options(${PROJECT_NAME})

if(BUILD_TESTING)
  # run linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # gtest
  set(OPTIMIZATION_TEST optimization_gtest)

  ament_add_gtest(${OPTIMIZATION_TEST}
          test/test_cache_states.hpp
          test/test_newton_optimization.hpp
          test/test_cache_states.cpp
#          test/test_newton_optimization.cpp
          test/test_more_thuente_line_search.cpp)
  autoware_set_compile_options(${OPTIMIZATION_TEST})
  target_compile_options(${OPTIMIZATION_TEST} PRIVATE -Wno-double-promotion -Wno-float-conversion)
  target_link_libraries(${OPTIMIZATION_TEST} ${PROJECT_NAME})
endif()

list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS "optimization-extras.cmake")
ament_auto_package()
