# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

cmake_minimum_required(VERSION 3.5)
project(autoware_rviz_plugins)

#dependencies
find_package(ament_cmake REQUIRED)
find_package(autoware_auto_cmake REQUIRED)
find_package(autoware_auto_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)

set(OD_PLUGIN_LIB_SRC
  src/object_detection/bounding_box_array_display.cpp
  src/planning/trajectory_display.cpp
)

set(OD_PLUGIN_LIB_HEADERS
  include/visibility_control.hpp
  include/object_detection/bounding_box_array_display.hpp
  include/planning/trajectory_display.hpp
)

add_library(${PROJECT_NAME} SHARED
  ${OD_PLUGIN_LIB_HEADERS}
  ${OD_PLUGIN_LIB_SRC}
)

target_link_libraries(${PROJECT_NAME}
  rviz_common::rviz_common
)
target_include_directories(${PROJECT_NAME} PRIVATE include)

autoware_set_compile_options(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} "rviz_common" "rviz_default_plugins" "autoware_auto_msgs")

# Settings to improve the build as suggested on https://github.com/ros2/rviz/blob/ros2/docs/plugin_development.md
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${PROJECT_NAME} PRIVATE "OBJECT_DETECTION_PLUGINS_BUILDING_LIBRARY")

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# Minimal amount of error suppression needed to build the rviz/qt related stuff
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-useless-cast
        -Wno-old-style-cast -Wno-double-promotion -Wno-pedantic -Wno-unused-parameter -Wno-overloaded-virtual)

if(BUILD_TESTING)
  # run linters
  autoware_static_code_analysis()
endif()

autoware_install(HAS_INCLUDE
        LIBRARIES ${PROJECT_NAME})

# Export the icons for the plugins
install(
  DIRECTORY "${CMAKE_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}"
)

ament_package()
