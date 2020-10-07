cmake_minimum_required(VERSION 3.5)
project(gnss_poser)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake)
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

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GeographicLib DEFAULT_MSG GeographicLib_LIBRARIES GeographicLib_INCLUDE_DIRS)
mark_as_advanced(GeographicLib_LIBRARIES GeographicLib_INCLUDE_DIRS)

###########
## Build ##
###########

include_directories(
  include
  ${GeographicLib_INCLUDE_DIRS}
)

# Generate exe file
set(GNSS_POSER_SRC
  src/gnss_poser_core.cpp
)
add_executable(gnss_poser_exe 
  src/gnss_poser_node.cpp
  ${GNSS_POSER_SRC}
)

# Add dependencies. use target_link_libaries() before Crystal
set(GNSS_POSER_DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geo_pos_conv
  ublox_msgs
)
ament_target_dependencies(gnss_poser_exe ${GNSS_POSER_DEPENDENCIES})

target_link_libraries(gnss_poser_exe
  GeographicLib_LIBRARIES
)

#############
## Install ##
#############

## Install executables and/or libraries
install(TARGETS gnss_poser_exe
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# set at the end of cmakelists
ament_package()
