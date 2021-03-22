# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

cmake_minimum_required(VERSION 3.5)
project(autoware_rviz_plugins)

#dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
ament_auto_find_build_dependencies()

set(OD_PLUGIN_LIB_SRC
  src/object_detection/bounding_box_array_display.cpp
  src/planning/trajectory_display.cpp
)

set(OD_PLUGIN_LIB_HEADERS
  include/visibility_control.hpp
)
set(OD_PLUGIN_LIB_HEADERS_TO_WRAP
  include/planning/trajectory_display.hpp
  include/object_detection/bounding_box_array_display.hpp
)

foreach(header "${OD_PLUGIN_LIB_HEADERS_TO_WRAP}")
  qt5_wrap_cpp(OD_PLUGIN_LIB_HEADERS_MOC "${header}")
endforeach()

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${OD_PLUGIN_LIB_HEADERS}
  ${OD_PLUGIN_LIB_HEADERS_MOC}
  ${OD_PLUGIN_LIB_SRC}
)
target_link_libraries(${PROJECT_NAME}
  rviz_common::rviz_common
  Qt5::Widgets
)
target_include_directories(${PROJECT_NAME} PRIVATE include)
autoware_set_compile_options(${PROJECT_NAME})

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
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export the icons for the plugins
ament_auto_package(INSTALL_TO_SHARE icons)
