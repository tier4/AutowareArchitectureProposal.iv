cmake_minimum_required(VERSION 3.5)
project(autoware_error_monitor)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-std=c++14 -Wno-unused-parameter -Wall -Wextra -Wpedantic 
  )
endif()

### Dependencies 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_system_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
# find_package(catkin REQUIRED COMPONENTS
#   autoware_system_msgs
#   diagnostic_msgs
#   roscpp
# )

find_package(fmt REQUIRED)

# catkin_package(
#   INCLUDE_DIRS
#     include
#   CATKIN_DEPENDS
#     autoware_system_msgs
#     diagnostic_msgs
# )

include_directories(
  include
)


# Target
## autoware_error_monitor_node
add_executable(autoware_error_monitor_exe
  src/autoware_error_monitor_node/autoware_error_monitor_node.cpp
  src/autoware_error_monitor_node/autoware_error_monitor_core.cpp)

# Add dependencies
set(AUTOWARE_ERROR_MONITOR_DEPENDENCIES
  rclcpp
  diagnostic_msgs
  autoware_system_msgs
)
ament_target_dependencies(autoware_error_monitor_exe ${AUTOWARE_ERROR_MONITOR_DEPENDENCIES} 
  PRIVATE fmt::fmt-header-only
)

# add_dependencies(autoware_error_monitor
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )

# Install
## executables and libraries
install(TARGETS ${PROJECT_NAME}_exe
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
)

## directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
