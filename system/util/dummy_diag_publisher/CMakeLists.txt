cmake_minimum_required(VERSION 3.5)
project(dummy_diag_publisher)

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

# find_package(fmt REQUIRED)

# generate_dynamic_reconfigure_options(
#   cfg/DummyDiagPublisher.cfg
# )

# catkin_package(
#   INCLUDE_DIRS
#     include
#   CATKIN_DEPENDS
#     diagnostic_updater
#     dynamic_reconfigure
# )

# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
# )

# Target
## dummy_diag_publisher_node
# add_executable(dummy_diag_publisher
#   src/dummy_diag_publisher_node/main.cpp
#   src/dummy_diag_publisher_node/dummy_diag_publisher_node.cpp
# )

### Target executable
set(DUMMY_DIAG_PUBLISHER_SRC
  src/dummy_diag_publisher_node/dummy_diag_publisher_node.cpp)

set(DUMMY_DIAG_PUBLISHER_HEADERS
  include/dummy_diag_publisher/dummy_diag_publisher_node.h)

ament_auto_add_executable(${PROJECT_NAME}
  src/dummy_diag_publisher_node/main.cpp
  ${DUMMY_DIAG_PUBLISHER_SRC}
  ${DUMMY_DIAG_PUBLISHER_HEADERS}
)

# target_link_libraries(dummy_diag_publisher
#   fmt
#   ${catkin_LIBRARIES}
# )

# add_dependencies(dummy_diag_publisher
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )

# Install
## executables and libraries
# install(
#   TARGETS
#     dummy_diag_publisher
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

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)

