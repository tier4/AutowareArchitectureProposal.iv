# Copyright 2019 the Autoware Foundation
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

# Look for GeographicLib
#
# Sets
#  GeographicLib_FOUND = TRUE
#  GeographicLib_INCLUDE_DIRS = /usr/local/include
#  GeographicLib_LIBRARIES = /usr/local/lib/libGeographic.so
#  GeographicLib_LIBRARY_DIRS = /usr/local/lib

find_library(GeographicLib_LIBRARIES NAMES Geographic
  PATHS "${CMAKE_INSTALL_PREFIX}/../GeographicLib/lib")

  if(GeographicLib_LIBRARIES)
  get_filename_component(GeographicLib_LIBRARY_DIRS
    "${GeographicLib_LIBRARIES}" PATH)

  get_filename_component(_ROOT_DIR "${GeographicLib_LIBRARY_DIRS}" PATH)
  set(GeographicLib_INCLUDE_DIRS "${_ROOT_DIR}/include")
  set(GeographicLib_BINARY_DIRS "${_ROOT_DIR}/bin")
  if(NOT EXISTS "${GeographicLib_INCLUDE_DIRS}/GeographicLib/Config.h")
    get_filename_component(_ROOT_DIR "${_ROOT_DIR}" PATH)                     # Added to script
    set(GeographicLib_INCLUDE_DIRS "${_ROOT_DIR}/include")                   # Added to script
    set(GeographicLib_BINARY_DIRS "${_ROOT_DIR}/bin")                        # Added to script
    if(NOT EXISTS "${GeographicLib_INCLUDE_DIRS}/GeographicLib/Config.h")    # Added to script
      unset(GeographicLib_INCLUDE_DIRS)
      unset(GeographicLib_LIBRARIES)
      unset(GeographicLib_LIBRARY_DIRS)
      unset(GeographicLib_BINARY_DIRS)
    endif()                                                                   # Added to script
  endif()
  unset(_ROOT_DIR)                                                           # Moved below if() statements
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GEOGRAPHICLIB DEFAULT_MSG
  GeographicLib_LIBRARY_DIRS GeographicLib_LIBRARIES GeographicLib_INCLUDE_DIRS)
mark_as_advanced(GeographicLib_LIBRARY_DIRS GeographicLib_LIBRARIES
  GeographicLib_INCLUDE_DIRS)
