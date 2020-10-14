cmake_minimum_required(VERSION 3.5)
project(gnss_poser)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto)
find_package(rclcpp)
find_package(tf2)
find_package(tf2_ros)
find_package(tf2_geometry_msgs)
find_package(std_msgs)
find_package(sensor_msgs)
find_package(geometry_msgs)
find_package(geo_pos_conv)
find_package(ublox_msgs)

find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h PATH_SUFFIXES GeographicLib)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})

find_library(GeographicLib_LIBRARIES NAMES Geographic)

ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(gnss_poser_node SHARED src/gnss_poser_core.cpp)

rclcpp_components_register_node(gnss_poser_node
  PLUGIN "GNSSPoser"
  EXECUTABLE gnss_poser
)

#############
## Install ##
#############

ament_auto_package(INSTALL_TO_SHARE
  launch
)
