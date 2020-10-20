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
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Target
## autoware_error_monitor_node
set(AUTOWARE_ERROR_MONITOR_SRC
  src/autoware_error_monitor_core.cpp
)

set(AUTOWARE_ERROR_MONITOR_HEADERS
  include/autoware_error_monitor/autoware_error_monitor_core.hpp
)

ament_auto_add_executable(${PROJECT_NAME}
  src/autoware_error_monitor_node.cpp
  ${AUTOWARE_ERROR_MONITOR_SRC}
  ${AUTOWARE_ERROR_MONITOR_HEADERS}
 )

# Install
## executables and libraries
ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
