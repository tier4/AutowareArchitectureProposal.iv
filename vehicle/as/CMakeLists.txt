cmake_minimum_required(VERSION 3.5)
project(as)
add_compile_options(-std=c++14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(automotive_platform_msgs REQUIRED)
find_package(automotive_navigation_msgs REQUIRED)
find_package(pacmod_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(AS_DEPENDENCIES
  rclcpp
  message_filters
  std_msgs
  geometry_msgs
  automotive_platform_msgs
  automotive_navigation_msgs
  pacmod_msgs
  autoware_control_msgs
  autoware_vehicle_msgs
)

add_executable(ssc_interface
  src/ssc_interface/ssc_interface_node.cpp
  src/ssc_interface/ssc_interface.cpp
)
ament_target_dependencies(ssc_interface ${AS_DEPENDENCIES})
ament_export_dependencies(${AS_DEPENDENCIES})


add_executable(pacmod_interface
  src/pacmod_interface/pacmod_interface.cpp
  src/pacmod_interface/pacmod_interface_node.cpp
)
ament_target_dependencies(pacmod_interface ${AS_DEPENDENCIES})
ament_export_dependencies(${AS_DEPENDENCIES})


install(TARGETS ssc_interface
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS pacmod_interface
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
