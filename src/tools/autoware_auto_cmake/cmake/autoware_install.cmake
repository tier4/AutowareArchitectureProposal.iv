# Copyright 2018 Apex.AI, Inc.
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

# Installs targets to be consistent for ROS 2
# :param LIBRARIES: list of cmake targets from add_libraries()
# :type LIBRARIES: a list of strings
# :param EXECUTABLES: a list of cmake targets from add_executables(),
#                     also gets exported
# :type EXECUTABLES: a list of executables
# :param HAS_CMAKE: true/false option on whether to install cmake folder
# :type HAS_CMAKE: an option
# :param HAS_INCLUDE: true/false;on/off option on whether to install
#                     include folder
# :type HAS_INCLUDE: an option
# :param HAS_LAUNCH: true/false;on/off option on whether to install
#                    launch folder
# :type HAS_LAUNCH: an option
macro(autoware_install)

  set(OPTION_NAMES "HAS_CMAKE;HAS_INCLUDE;HAS_LAUNCH;HAS_PARAM")
  set(ARGN_NAMES "LIBRARIES;EXECUTABLES")

  cmake_parse_arguments(ARG
    "${OPTION_NAMES}"
    ""
    "${ARGN_NAMES}"
    ${ARGN})

  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "autoware_install called with unused arguments: " ${ARG_UNPARSED_ARGUMENTS})
  endif()

  if(ARG_LIBRARIES)
    # install libraries
    install(
      TARGETS ${ARG_LIBRARIES}
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin)
    foreach(LIB ${ARG_LIBRARIES})
      ament_export_libraries(${LIB})
    endforeach()
  endif()

  if(ARG_EXECUTABLES)
    # install executables
    foreach(EXE ${ARG_EXECUTABLES})
      install(
        TARGETS ${EXE} EXPORT ${EXE}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})
    endforeach()
  endif()

  if(ARG_HAS_CMAKE)
    # For cmake folder
    install(
      DIRECTORY cmake
      DESTINATION share/${PROJECT_NAME}
    )
  endif()

  if(ARG_HAS_INCLUDE)
    # For include directories
    install(
      DIRECTORY include/
      DESTINATION include
    )
    ament_export_include_directories(include)
  endif()

  if(ARG_HAS_LAUNCH)
    # For launch directories
    install(
      DIRECTORY launch/
      DESTINATION share/${PROJECT_NAME}/
    )
  endif()

  if(ARG_HAS_PARAM)
    # For configuration files in the param folder
    install(
        DIRECTORY param/
        DESTINATION share/${PROJECT_NAME}/
    )
  endif()
endmacro()
