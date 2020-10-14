cmake_minimum_required(VERSION 3.0.2)
project(object_merger)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  message_filters
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(object_association_merger_node
  src/object_association_merger/utils/utils.cpp
  src/object_association_merger/data_association/data_association.cpp
  src/object_association_merger/data_association/successive_shortest_path/successive_shortest_path.cpp
  src/object_association_merger/node.cpp
  src/object_association_merger/main.cpp
)

add_dependencies(object_association_merger_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(object_association_merger_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
  TARGETS
    object_association_merger_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
