cmake_minimum_required(VERSION 3.0.2)
project(velodyne_monitor)

## Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  roslint
  roscpp
  std_msgs
)

find_package(fmt REQUIRED)

set(ROSLINT_CPP_OPTS "--filter=-build/c++14")
roslint_cpp()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    std_msgs
    diagnostic_msgs
    diagnostic_updater
)

###########
## Build ##
###########

## Specify additional locations of header files

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(velodyne_monitor
  src/velodyne_monitor_node.cpp
  src/velodyne_monitor.cpp
)

## Add cmake target dependencies of the executable
add_dependencies(velodyne_monitor ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(velodyne_monitor ${catkin_LIBRARIES} cpprest crypto fmt)


#############
## Install ##
#############

## Mark executables for installation
install(
  TARGETS
    velodyne_monitor
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install launch
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
