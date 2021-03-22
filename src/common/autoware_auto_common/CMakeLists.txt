# Copyright 2019 the Autoware Foundation
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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
cmake_minimum_required(VERSION 3.5)

### Export headers
project(autoware_auto_common)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Temporarily disable cpplint and uncrustify
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )

  ament_lint_auto_find_test_dependencies()

  file(GLOB_RECURSE FILES_MINUS_SOME
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*
    ${CMAKE_CURRENT_SOURCE_DIR}/test/*
  )

  list(FILTER FILES_MINUS_SOME EXCLUDE REGEX ".*filesystem.hpp")

  # Re-enable cpplint
  set(ament_cmake_cpplint_FOUND FALSE)
  find_package(ament_cmake_cpplint)
  ament_cpplint(${FILES_MINUS_SOME})

  # Re-enable uncrustify
  set(ament_cmake_uncrustify_FOUND FALSE)
  find_package(ament_cmake_uncrustify)
  ament_uncrustify(${FILES_MINUS_SOME})

  # Unit tests
  set(TEST_COMMON test_common_gtest)
  ament_add_gtest(${TEST_COMMON}
          test/gtest_main.cpp
          test/test_float_comparisons.cpp
          test/test_bool_comparisons.cpp
          test/test_byte_reader.cpp
          test/test_type_traits.cpp
          test/test_type_name.cpp
          test/test_message_field_adapters.cpp)
  target_include_directories(${TEST_COMMON} PRIVATE include)
  ament_target_dependencies(${TEST_COMMON} builtin_interfaces)
endif()

# Ament Exporting
ament_auto_package()
