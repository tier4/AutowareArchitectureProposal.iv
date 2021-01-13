cmake_minimum_required(VERSION 3.0.2)
project(multi_object_tracker)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  autoware_utils
  roscpp
  tf2
  tf2_ros
  kalman_filter
  unique_id
  uuid_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    autoware_perception_msgs
    autoware_utils
    roscpp
    tf2
    tf2_ros
    kalman_filter
    unique_id
    uuid_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(multi_object_tracker_node
  src/main.cpp
  src/node.cpp
  src/utils/utils.cpp
  src/tracker/model/tracker_base.cpp
  src/tracker/model/big_vehicle_tracker.cpp
  src/tracker/model/normal_vehicle_tracker.cpp
  src/tracker/model/multiple_vehicle_tracker.cpp
  src/tracker/model/bicycle_tracker.cpp
  src/tracker/model/pedestrian_tracker.cpp
  src/tracker/model/pedestrian_and_bicycle_tracker.cpp
  src/data_association/data_association.cpp
  src/data_association/successive_shortest_path/successive_shortest_path.cpp
)

add_dependencies(multi_object_tracker_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(multi_object_tracker_node ${catkin_LIBRARIES})

install(TARGETS multi_object_tracker_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
