cmake_minimum_required(VERSION 3.5)
project(emergency_handler)

### Compile options
# add_compile_options(-std=c++14)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

### Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_control_msgs)
find_package(autoware_system_msgs)
find_package(autoware_vehicle_msgs)
find_package(geometry_msgs)
find_package(std_msgs)
# find_package(catkin REQUIRED COMPONENTS
#   autoware_control_msgs
#   autoware_system_msgs
#   autoware_vehicle_msgs
#   geometry_msgs
#   roscpp
#   std_msgs
# )

# Add path of include dir
# catkin_package(
#   INCLUDE_DIRS
#     include
#   CATKIN_DEPENDS
#     autoware_control_msgs
#     autoware_system_msgs
#     autoware_vehicle_msgs
#     geometry_msgs
#     std_msgs
# )

include_directories(include)

### Target executable
add_executable(${PROJECT_NAME}_exe
  src/emergency_handler_node/emergency_handler_node.cpp
  src/emergency_handler_node/emergency_handler_core.cpp
)

# Add dependencies. use target_link_libaries() before Crystal
set(EMERGENCY_HANDLER_DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  autoware_system_msgs
  autoware_control_msgs
  autoware_vehicle_msgs
)
ament_target_dependencies(${PROJECT_NAME}_exe ${EMERGENCY_HANDLER_DEPENDENCIES})

# target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )

# add_dependencies(${PROJECT_NAME}
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )

## Install executables and/or libraries
install(TARGETS ${PROJECT_NAME}_exe
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

# install(
#   TARGETS
#     ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# ## project namespaced headers
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# ## directories
# install(
#   DIRECTORY
#     launch
#     config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
