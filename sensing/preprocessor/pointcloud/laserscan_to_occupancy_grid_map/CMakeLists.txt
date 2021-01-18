cmake_minimum_required(VERSION 3.0.2)
project(laserscan_to_occupancy_grid_map)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  sensor_msgs
  laser_geometry
  nav_msgs
  pcl_ros
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  visualization_msgs
  costmap_2d
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES laserscan_to_occupancy_grid_map
  CATKIN_DEPENDS roscpp nodelet sensor_msgs laser_geometry nav_msgs pcl_ros tf2 tf2_ros tf2_sensor_msgs tf2_geometry_msgs visualization_msgs costmap_2d

)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(laserscan_to_occupancy_grid_map
  src/laserscan_to_occupancy_grid_map_nodelet.cpp
  src/occupancy_grid_map.cpp
  src/updater/occupancy_grid_map_binary_bayes_filter_updater.cpp
)
target_link_libraries(laserscan_to_occupancy_grid_map ${catkin_LIBRARIES})

add_executable(laserscan_to_occupancy_grid_map_node src/laserscan_to_occupancy_grid_map_node.cpp)
target_link_libraries(laserscan_to_occupancy_grid_map_node laserscan_to_occupancy_grid_map ${catkin_LIBRARIES})

install(TARGETS laserscan_to_occupancy_grid_map laserscan_to_occupancy_grid_map_node
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
