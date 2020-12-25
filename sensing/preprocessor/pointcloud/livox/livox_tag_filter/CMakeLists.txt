cmake_minimum_required(VERSION 3.0.2)
project(livox_tag_filter)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    pcl_ros
    sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Target
## livox_tag_filter_node
add_executable(livox_tag_filter_node
  src/livox_tag_filter_node/main.cpp
  src/livox_tag_filter_node/livox_tag_filter_node.cpp
)

target_link_libraries(livox_tag_filter_node
  ${catkin_LIBRARIES}
)

add_dependencies(livox_tag_filter_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Install
## executables and libraries
install(
  TARGETS
    livox_tag_filter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## directories
install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
