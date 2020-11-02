cmake_minimum_required(VERSION 3.5)
project(autoware_state_monitor)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# find_package(rclcpp REQUIRED)
# find_package(autoware_planning_msgs REQUIRED)
# find_package(autoware_system_msgs REQUIRED)
# find_package(autoware_vehicle_msgs REQUIRED)
# # find_package(diagnostic_updater REQUIRED)
# # find_package(topic_tools REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(fmt REQUIRED)

include_directories(include)

# Target
## Set source files
set(AUTOWARE_STATE_MONITOR_SRC
  src/autoware_state_monitor_node/autoware_state_monitor_node.cpp
  src/autoware_state_monitor_node/state_machine.cpp
  src/autoware_state_monitor_node/diagnostics.cpp
)

## Add executables
ament_auto_add_executable(autoware_state_monitor 
  src/autoware_state_monitor_node/main.cpp
  ${AUTOWARE_STATE_MONITOR_SRC}
)

# set(AUTOWARE_STATE_MONITOR_DEPENDENCIES
#   rclcpp
#   autoware_planning_msgs
#   autoware_system_msgs
#   autoware_vehicle_msgs
#   # diagnostic_updater
#   # topic_tools
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
# )

# ## Add dependencies
# ament_target_dependencies(autoware_state_monitor_exe 
#   ${AUTOWARE_STATE_MONITOR_DEPENDENCIES} 
#   PRIVATE fmt::fmt-header-only
# )

# # Install
# ## Install executables and/or libraries
# install(TARGETS ${PROJECT_NAME}_exe
#   ARCHIVE DESTINATION lib/${PROJECT_NAME}
#   LIBRARY DESTINATION lib/${PROJECT_NAME}
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

# ## Install project namespaced headers
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include)

# install(DIRECTORY config launch
#   DESTINATION share/${PROJECT_NAME}
# )

# # set at the end of cmakelists
# ament_package()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
