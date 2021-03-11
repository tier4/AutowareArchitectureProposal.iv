# Copyright 2018 the Autoware Foundation
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

# Default to C11 (rcutils uses C11 thread local storage)
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
# Clang tidy
if(TIDY_WITH_CLANG)
  string(CONCAT CMAKE_CXX_CLANG_TIDY
    "clang-tidy;"
    "-checks=-*,"
    "bugprone-*,"
    "cert-*,"
    "cppcoreguidelines-*,"
    "clang-analyze-*,"
    "google-*,"
    "hicpp-*,"
    "modernize-*,"
    "performance-*,"
    "readability-*")
  message(${CMAKE_CXX_CLANG_TIDY})
endif()

# Try to adhere to strict ISO C++ as much as possible:
#    from https://lefticus.gitbooks.io/cpp-best-practices/content/02-Use_the_Tools_Available.html
function(autoware_set_compile_options target)
if(WIN32)
  # Causes the visibility macros to use dllexport rather than dllimport,
  # which is appropriate when building the dll but not consuming it.
  string(TOUPPER ${target} PROJECT_NAME_UPPER)
  target_compile_definitions(${target} PRIVATE ${PROJECT_NAME_UPPER}_BUILDING_DLL)
  target_compile_options(${target} PRIVATE "/bigobj")
  add_definitions(-D_CRT_NONSTDC_NO_WARNINGS)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
  add_definitions(-D_WINSOCK_DEPRECATED_NO_WARNINGS)
else()
  target_compile_options(${target} PRIVATE
    -Wall
    -Werror
    -Wextra
    #-Wshadow             # causes issues with ROS 2 headers
    #-Wnon-virtual-dtor   # causes issues with ROS 2 headers
    -pedantic
    -Wcast-align
    -Wunused
    -Wconversion
    -Wsign-conversion
    -Wdouble-promotion
    #-Wnull-dereference    # gcc6
    #-Wduplicated-branches # gcc7
    #-Wduplicated-cond     # gcc6
    #-Wrestrict            # gcc7
    -fvisibility=hidden)
  # C++-only options
  target_compile_options(${target}
    PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -Woverloaded-virtual -Wold-style-cast>)

  if(NOT APPLE)
    # GCC/G++ Only, not CLang
    target_compile_options(${target}
      PUBLIC $<$<COMPILE_LANGUAGE:CXX>: -Wuseless-cast>)
    target_compile_options(${target} PRIVATE -Wlogical-op -frecord-gcc-switches)
  endif()

  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
      set_target_properties(${target} PROPERTIES COMPILE_FLAGS "-Og")
  else()
      set_target_properties(${target} PROPERTIES COMPILE_FLAGS "-O3 -ftree-vectorize")
  endif()
endif()
endfunction()
