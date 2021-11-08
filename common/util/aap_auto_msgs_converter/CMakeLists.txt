cmake_minimum_required(VERSION 3.5)
project(aap_auto_msgs_converter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(aap_auto_msgs_converter SHARED
  src/aap_auto_perception_msgs_converter.cpp
)
rclcpp_components_register_node(aap_auto_msgs_converter
  PLUGIN "aap_auto_msgs_converter::AAPAutoPerceptionMsgsConverter"
  EXECUTABLE aap_auto_perception_msgs_converter_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
