cmake_minimum_required(VERSION 3.5)
project(gyro_odometer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# find_package(roscpp REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)

# catkin_package(
#   INCLUDE_DIRS include
# )

set(GRYO_ODOMETER_SRC
  src/gyro_odometer_core.cpp)

set(GRYO_ODOMETER_HEADERS
  include/gyro_odometer/gyro_odometer_core.h)

ament_auto_add_library(${PROJECT_NAME} 
  src/gyro_odometer_node.cpp
  ${GRYO_ODOMETER_SRC}
  ${GRYO_ODOMETER_HEADERS}
 )

# include_directories(include ${catkin_INCLUDE_DIRS})

# add_executable(gyro_odometer
#         src/gyro_odometer_node.cpp
#         src/gyro_odometer_core.cpp
# )

# add_dependencies(gyro_odometer ${${PROJECT_NAME}_EXPORTED_TARGETS}
#                                   ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries(gyro_odometer ${catkin_LIBRARIES})

# install(TARGETS
#             gyro_odometer
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#         )

# install(
#   DIRECTORY
#     launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

ament_auto_package(
  INSTALL_TO_SHARE
)
