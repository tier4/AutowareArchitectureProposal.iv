cmake_minimum_required(VERSION 3.5)
project(autoware_debug_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bool.msg"
  "msg/Float32.msg"
  "msg/Float32MultiArray.msg"
  "msg/Float64.msg"
  "msg/Float64MultiArray.msg"
  "msg/Int32.msg"
  "msg/Int32MultiArray.msg"
  "msg/Int64.msg"
  "msg/Int64MultiArray.msg"
  "msg/String.msg"
  DEPENDENCIES builtin_interfaces
)

ament_package()
